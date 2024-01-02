from collections import deque
import logging
import ntcore as nt
import numpy as np

from data_utils import Odometry, Vision, Fiducial, Measurement

logger = logging.getLogger("NetworkTablesManager")
logging.basicConfig(level=logging.INFO)


class NTManager:
    def __init__(self, server=None):
        logger.info("Initializing NetworkTables...")
        self.inst = nt.NetworkTableInstance.getDefault()
        self.inst.setServer(server_name=server)
        self.inst.startClient4("quixsam")

        quixsam_table = self.inst.getTable("localizer")

        # Setup topics
        self.targets_sub = quixsam_table.getDoubleArrayTopic("targets").subscribe(
            [], nt.PubSubOptions(sendAll=True)
        )
        self.measurements_sub = quixsam_table.getDoubleArrayTopic(
            "measurements"
        ).subscribe([], nt.PubSubOptions(sendAll=True))
        self.estimates_pub = quixsam_table.getDoubleArrayTopic("estimates").publish(
            nt.PubSubOptions(sendAll=True)
        )

        # Buffer to read measurements into
        self.last_measurement_id = -1  # Enforce IDs be always increasing
        self.measurement_buffer = deque()

        # This should only be read once
        self.retroreflective_targets = []
        self.apriltag_targets = {}

        # Keep track of the last returned ID so we can rate limit what gets sent to the particle filter
        self.last_returned_id = 0
        self.last_vision_id = 0

    def is_connected(self):
        return self.inst.isConnected()

    def clear_table(self, table):
        for key in table.getKeys():
            table.delete(key)

    def measurement_received(self, value):
        ODOM_DATA_LENGTH = 6
        VISION_DATA_LENGTH = 7
        try:
            id_ = int(value[0])
            if id_ <= self.last_measurement_id:
                logging.critical(f"Measurement ID decreased")
            elif (
                all(np.isfinite(value))
                and (len(value) - ODOM_DATA_LENGTH - 1) % VISION_DATA_LENGTH == 0
            ):
                vision = []
                for i in range(
                    int((len(value) - 1 - ODOM_DATA_LENGTH) / VISION_DATA_LENGTH)
                ):
                    vision.append(
                        Vision(
                            int(value[ODOM_DATA_LENGTH + VISION_DATA_LENGTH * i + 1]),
                            int(value[ODOM_DATA_LENGTH + VISION_DATA_LENGTH * i + 2]),
                            int(value[ODOM_DATA_LENGTH + VISION_DATA_LENGTH * i + 3]),
                            value[ODOM_DATA_LENGTH + VISION_DATA_LENGTH * i + 4],
                            value[ODOM_DATA_LENGTH + VISION_DATA_LENGTH * i + 5],
                        )
                    )
                self.measurement_buffer.append(
                    Measurement(
                        int(value[0]), Odometry(value[1], value[2], value[3]), vision
                    )
                )
                self.last_measurement_id = id_
            else:
                logging.critical(f"Invalid measurement data: {value}")
        except ValueError:
            logging.critical(f"Invalid measurement data")

    def get_targets(self):
        DATA_LENGTH = 8
        for data in self.targets_sub.readQueue():
            try:
                for i in range(int(len(data.value) / DATA_LENGTH)):
                    id = data.value[DATA_LENGTH * i]
                    target = Fiducial(
                        id,
                        data.value[DATA_LENGTH * i + 1],
                        data.value[DATA_LENGTH * i + 2],
                        data.value[DATA_LENGTH * i + 3],
                        data.value[DATA_LENGTH * i + 4],
                        data.value[DATA_LENGTH * i + 5],
                        data.value[DATA_LENGTH * i + 6],
                        data.value[DATA_LENGTH * i + 7],
                    )
                    if id == -1:
                        self.retroreflective_targets.append(target)
                    else:
                        self.apriltag_targets[id] = target
            except ValueError:
                logging.critical("Invalid targets data.")
        return self.retroreflective_targets, self.apriltag_targets

    def get_next(self):
        for data in self.measurements_sub.readQueue():
            self.measurement_received(data.value)

        logging.debug(f"Buffer size: {len(self.measurement_buffer)}")

        next_id = None
        next_odom = None
        next_vision = None
        if len(self.measurement_buffer) > 0:
            measurement = self.measurement_buffer.popleft()
            next_id = measurement.id
            next_odom = measurement.odom
            next_vision = measurement.vision if len(measurement.vision) > 0 else None

        # Exit early if there is no data
        if next_id is None:
            return None, None, None

        # Rate limit the output, guaranteeing we have at least one vision measurement every
        # NUM_TO_SKIP IDs as long as it is available.
        # Note that in the worst case we may output twice (one odom only one odom + vision)
        # for every NUM_TO_SKIP rather than once.
        NUM_TO_SKIP = 5
        if next_vision is not None and self.last_vision_id + NUM_TO_SKIP < next_id:
            self.last_returned_id = next_id
            self.last_vision_id = next_id
            return next_id, next_odom, next_vision

        if self.last_returned_id + NUM_TO_SKIP < next_id:
            self.last_returned_id = next_id
            return next_id, next_odom, next_vision

        return None, None, None

    def publish_estimate(
        self, id_: int, x: float, y: float, theta: float, has_vision: bool
    ):
        self.estimates_pub.set([float(id_), x, y, theta, 1.0 if has_vision else 0.0])
