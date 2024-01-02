from collections import deque
import logging
import ntcore as nt
import numpy as np

from data_utils import Odometry, Vision, Fiducial

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
        self.odometry_sub = quixsam_table.getDoubleArrayTopic("odometry").subscribe(
            [], nt.PubSubOptions(sendAll=True)
        )
        self.vision_sub = quixsam_table.getDoubleArrayTopic("vision").subscribe(
            [], nt.PubSubOptions(sendAll=True)
        )
        self.estimates_pub = quixsam_table.getDoubleArrayTopic("estimates").publish(
            nt.PubSubOptions(sendAll=True)
        )

        # Buffers to read data into
        self.last_odometry_id = -1  # Enforce IDs be always increasing
        self.odometry_buffer = deque()
        self.last_vision_id = -1
        self.vision_buffer = deque()

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

    def odometry_received(self, value):
        try:
            id_ = int(value[0])
            if id_ <= self.last_odometry_id:
                logging.critical(f"Odometry ID decreased")
            elif np.isfinite(id_) and all(np.isfinite(value)) and len(value) == 7:
                self.odometry_buffer.append(
                    Odometry(
                        id_,
                        value[1],
                        value[2],
                        value[3],
                    )
                )
                self.latest_odometry_id = id_
            else:
                logging.critical(f"Invalid odometry data: {value}")
        except ValueError:
            logging.critical(f"Invalid odometry data")

    def vision_received(self, value):
        DATA_LENGTH = 7
        try:
            id_ = int(value[0])
            if id_ <= self.last_vision_id:
                logging.critical(f"Vision ID decreased")
            elif (
                np.isfinite(id_)
                and all(np.isfinite(value))
                and (len(value) - 1) % DATA_LENGTH == 0
            ):
                data = []
                for i in range(int((len(value) - 1) / DATA_LENGTH)):
                    data.append(
                        Vision(
                            id_,
                            int(value[DATA_LENGTH * i + 1]),
                            int(value[DATA_LENGTH * i + 2]),
                            int(value[DATA_LENGTH * i + 3]),
                            value[DATA_LENGTH * i + 4],
                            value[DATA_LENGTH * i + 5],
                        )
                    )
                self.vision_buffer.append(data)
                self.latest_vision_id = id_
            else:
                logging.critical(f"Invalid vision data: {value}")
        except ValueError:
            logging.critical(f"Invalid vision data")

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
        for data in self.odometry_sub.readQueue():
            self.odometry_received(data.value)
        for data in self.vision_sub.readQueue():
            self.vision_received(data.value)

        logging.debug(
            f"Buffer sizes: {len(self.odometry_buffer)}, {len(self.vision_buffer)}"
        )

        next_odom = None
        next_vision = None
        if len(self.odometry_buffer) > 0:
            # Return the next odometry if it exists.
            next_odom = self.odometry_buffer.popleft()

            # Return the the next vision if it exists and has the same ID as odometry.
            # Flush all vision older than this odom message.
            while len(self.vision_buffer) > 0 and (
                len(self.vision_buffer[0]) == 0  # Flush null packets as well
                or self.vision_buffer[0][0].id <= next_odom.id
            ):
                if len(self.vision_buffer[0]) == 0:
                    self.vision_buffer.popleft()
                else:
                    next_vision = self.vision_buffer.popleft()

        # Exit early if there is no data
        if next_odom is None:
            return None, None

        # Sanity check that the IDs match. It's better to throw away vision data than have an
        # incorrect time sync, but ideally we should never have to do this.
        if next_vision is not None and next_odom.id != next_vision[0].id:
            logging.warning(
                f"Odom and vision ID don't match: {next_odom.id}, {next_vision[0].id}"
            )
            next_vision = None

        # Rate limit the output, guaranteeing we have at least one vision measurement every
        # NUM_TO_SKIP IDs as long as it is available.
        # Note that in the worst case we may output twice (one odom only one odom + vision)
        # for every NUM_TO_SKIP rather than once.
        NUM_TO_SKIP = 5
        if next_vision is not None and self.last_vision_id + NUM_TO_SKIP < next_odom.id:
            self.last_vision_id = next_odom.id
            self.last_returned_id = next_odom.id
            return next_odom, next_vision

        if self.last_returned_id + NUM_TO_SKIP < next_odom.id:
            self.last_returned_id = next_odom.id
            return next_odom, next_vision

        return None, None

    def publish_estimate(
        self, id_: int, x: float, y: float, theta: float, has_vision: bool
    ):
        self.estimates_pub.set([float(id_), x, y, theta, 1.0 if has_vision else 0.0])
