import logging
import time

import ntcore as nt

logger = logging.getLogger("ScoringSelector: NTHelper")
logging.basicConfig(level=logging.INFO)


class NTHelper:
    def __init__(self, server="localhost"):
        self.server = server
        logger.info("Initializing NetworkTables...")
        self.inst = nt.NetworkTableInstance.getDefault()
        self.inst.setServer(server_name=server)
        self.inst.startClient4("scoring-selector")

        # Extra check for real robot
        if server != "localhost":
            while not self.inst.isConnected():
                logger.info("Waiting for robot connection...")
                time.sleep(1.0)

        table = self.inst.getTable("scoring-selector")
        self.loc_pub = table.getIntegerArrayTopic("loc").publish(
            nt.PubSubOptions(sendAll=True)
        )
        self.loc_sub = table.getIntegerArrayTopic("loc").subscribe(
            [-1, -1], nt.PubSubOptions(sendAll=True)
        )

        self.last_grid_id = -1
        self.last_node_id = -1

    def set_scoring_location(self, grid_id, node_id):
        """
        From the driver's perspective,
        Grid IDs are 0-2 from left to right
        Node IDs within a grid are labled as such:
        |0|1|2| (bottom row)
        |3|4|5| (middle row)
        |6|7|8| (top row)
        """
        if grid_id < 0 or grid_id > 2:
            logger.error(f"Invalid grid_id: {grid_id}")
            return
        if node_id < 0 or node_id > 8:
            logger.error(f"Invalid node_id: {node_id}")
            return

        if self.last_grid_id != grid_id or self.last_node_id != node_id:
            self.loc_pub.set([grid_id, node_id])
            self.last_grid_id = grid_id
            self.last_node_id = node_id
            logger.info(f"Selected: ({grid_id}, {node_id})")

    def get_scoring_location(self):
        loc = self.loc_sub.get()
        if len(loc) != 2:
            return
        grid_id, node_id = loc

        # Bounds check.
        if grid_id < 0 or grid_id > 2 or node_id < 0 or node_id > 8:
            return

        return grid_id, node_id
