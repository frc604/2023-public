from turtle import title
import matplotlib.pyplot as plt
from multiprocessing import Process, Manager, Queue
import os
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import transformations as tf
import pyqtgraph as pg

from data_utils import *
from helpers import in2m
from particle_filter import (
    ROBOT_HALF_LENGTH,
    ROBOT_HALF_WIDTH,
    CAMERAS_TRANSFORMS,
    FIELD_LENGTH,
    FIELD_WIDTH,
)

FIELD_IMAGE = os.path.join(os.path.dirname(__file__), "../../images/field.png")

ROBOT_COORDS = np.array(
    [
        [
            ROBOT_HALF_LENGTH,
            -ROBOT_HALF_LENGTH,
            -ROBOT_HALF_LENGTH,
            ROBOT_HALF_LENGTH,
            ROBOT_HALF_LENGTH,
        ],
        [
            ROBOT_HALF_WIDTH,
            ROBOT_HALF_WIDTH,
            -ROBOT_HALF_WIDTH,
            -ROBOT_HALF_WIDTH,
            ROBOT_HALF_WIDTH,
        ],
        [1.0, 1.0, 1.0, 1.0, 1.0],
    ]
)

LL_TRANSLATION = tf.translation_from_matrix(CAMERAS_TRANSFORMS[0])
LL_FOV_LENGTH = 3  # m
LL_FOV = np.deg2rad(59.6)  # degrees

LL_CORDS = np.array(
    [
        [
            LL_TRANSLATION[0],
            LL_TRANSLATION[0] + LL_FOV_LENGTH,
            LL_TRANSLATION[0] + LL_FOV_LENGTH,
            LL_TRANSLATION[0],
        ],
        [
            LL_TRANSLATION[1],
            LL_TRANSLATION[1] + LL_FOV_LENGTH * np.tan(LL_FOV / 2),
            LL_TRANSLATION[1] - LL_FOV_LENGTH * np.tan(LL_FOV / 2),
            LL_TRANSLATION[1],
        ],
        [1.0, 1.0, 1.0, 1.0],
    ]
)

LL_BEARING_COORDS = np.array([[LL_FOV_LENGTH, 0.0], [0.0, 0.0], [1.0, 1.0]])

LL_FOV_PEN = pg.mkPen(color=pg.mkColor(63, 191, 63, 90), width=2)
LL_FILL_BRUSH = pg.mkBrush(63, 191, 63, 50)
BEARING_RETRO_PEN = pg.mkPen(color=pg.mkColor(63, 191, 63, 255), width=4)
BEARING_TAG_PEN = pg.mkPen(color=pg.mkColor(255, 0, 255, 255), width=2)
BEARING_TAG_CORNER_PEN = pg.mkPen(color=pg.mkColor(0, 255, 255, 255), width=2)


class CenteredArrowItem(pg.ArrowItem):
    def paint(self, p, *args):
        p.translate(-self.boundingRect().center())
        pg.ArrowItem.paint(self, p, *args)


class Plotter:
    def __init__(self, num_particles, is_red=True):
        self.num_particles = num_particles
        self.is_red = is_red

    def run(self):
        self.win = pg.GraphicsLayoutWidget(title="QuixPF", show=True)
        self.win.resize(1000, 500)

        # Create 2D plotting area
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked()

        # Show field
        img = pg.ImageItem()
        img.setImage(np.rot90(plt.imread(FIELD_IMAGE), k=3 if self.is_red else 1))
        img.setRect(0, 0, FIELD_LENGTH, FIELD_WIDTH)
        self.plot.addItem(img)

        # Initialize scatter plot for targets
        self.retroreflective_targets = pg.ScatterPlotItem()
        self.retroreflective_targets.setData(
            size=0.1, brush=pg.mkColor(0, 255, 0, 255), pxMode=False
        )
        self.plot.addItem(self.retroreflective_targets)

        self.apriltag_targets = pg.ScatterPlotItem()
        self.apriltag_targets.setData(
            size=0.1, brush=pg.mkColor(255, 0, 255, 255), pxMode=False
        )
        self.plot.addItem(self.apriltag_targets)

        # Initialize scatter plot for particles
        self.scatter = pg.ScatterPlotItem(size=self.num_particles)
        self.scatter.setData(size=1, pen=pg.mkPen(None))
        self.plot.addItem(self.scatter)

        # Initialize arrow for best estimate
        self.arrow = CenteredArrowItem()
        self.plot.addItem(self.arrow)

        # Initialize robot plotting of best estimate
        self.robot = pg.PlotCurveItem(
            ROBOT_COORDS[0, :],
            ROBOT_COORDS[1, :],
            antialias=True,
            pen=pg.mkPen("k", width=4),
        )
        self.plot.addItem(self.robot)

        self.ll_top = pg.PlotDataItem(
            [LL_CORDS[0, 0], LL_CORDS[0, 1]],
            [LL_CORDS[1, 0], LL_CORDS[1, 1]],
            antialias=True,
            pen=LL_FOV_PEN,
        )
        self.plot.addItem(self.ll_top)

        self.ll_bottom = pg.PlotDataItem(
            [LL_CORDS[0, 0], LL_CORDS[0, 2]],
            [LL_CORDS[1, 0], LL_CORDS[1, 2]],
            antialias=True,
            pen=LL_FOV_PEN,
        )
        self.plot.addItem(self.ll_bottom)

        self.ll_fill = pg.FillBetweenItem(
            self.ll_bottom, self.ll_top, brush=LL_FILL_BRUSH
        )
        self.ll_fill.setZValue(1)
        self.plot.addItem(self.ll_fill)

        # By target id
        self.ll_bearings = {}

        timer = QtCore.QTimer()
        timer.timeout.connect(self._update)
        timer.start(int(1000.0 / 60.0))

        pg.exec()

    def start(self):
        self.q = Queue()
        self.targets_q = Queue()
        self.p = Process(target=self.run)
        self.p.start()
        return self.q, self.targets_q

    def join(self):
        return self.p.join()

    def _update(self):
        # Update targets
        latest_targets = None
        while not self.targets_q.empty():
            retroreflective_targets, apriltag_targets = self.targets_q.get()
            if retroreflective_targets:
                retroreflective_data = np.array(retroreflective_targets)
                self.retroreflective_targets.setData(
                    x=retroreflective_data[:, 0],
                    y=retroreflective_data[:, 1],
                )
            if apriltag_targets:
                apriltag_data = np.array(list(apriltag_targets.values()))
                self.apriltag_targets.setData(
                    x=apriltag_data[:, 0],
                    y=apriltag_data[:, 1],
                )

        # Fast fowrard to the latest element
        latest = None
        while not self.q.empty():
            latest = self.q.get()

        if latest is not None:
            vision, particles, best_estimate, has_vision = latest

            # Particles
            if particles is not None:
                NUM_TO_PLOT = 1000
                self.scatter.setData(
                    x=particles[:NUM_TO_PLOT, 0, 3],
                    y=particles[:NUM_TO_PLOT, 1, 3],
                    brush="g" if has_vision else "r",
                )

            x, y, theta = best_estimate

            # Update arrow
            self.arrow.setPos(x, y)
            self.arrow.setStyle(angle=-np.rad2deg(theta) + 180)

            # Update robot
            tr = QtGui.QTransform()
            tr.translate(x, y)
            tr.rotateRadians(theta)
            m = pg.transformToArray(tr)
            coords = np.dot(m, ROBOT_COORDS)
            self.robot.setData(coords[0, :], coords[1, :])

            # Draw LL FOV
            if has_vision:
                self.ll_top.setPen(LL_FOV_PEN)
                self.ll_bottom.setPen(LL_FOV_PEN)
                self.ll_fill.setBrush(LL_FILL_BRUSH)

                tr = QtGui.QTransform()
                tr.translate(x, y)
                tr.rotateRadians(theta)
                m = pg.transformToArray(tr)
                coords = np.dot(m, LL_CORDS)
                self.ll_top.setData(
                    [coords[0, 0], coords[0, 1]], [coords[1, 0], coords[1, 1]]
                )
                self.ll_bottom.setData(
                    [coords[0, 0], coords[0, 2]], [coords[1, 0], coords[1, 2]]
                )
            else:
                self.ll_top.setPen(pg.mkPen(color=pg.mkColor(0, 0, 0, 0)))
                self.ll_bottom.setPen(pg.mkPen(color=pg.mkColor(0, 0, 0, 0)))
                self.ll_fill.setBrush(pg.mkBrush(0, 0, 0, 0))

            # Draw measurement bearing
            detected_ids = set()
            if vision:
                for i, v in enumerate(vision):
                    # Add if we haven't seen this camearID + target ID + cornerID before
                    id = f"{v.cameraID}-{-1 - i if v.targetID == -1 else v.targetID}-{v.targetCornerID}"
                    isRetro = v.targetID == -1

                    if id not in self.ll_bearings:
                        self.ll_bearings[id] = pg.PlotDataItem(
                            [LL_BEARING_COORDS[0, 0], LL_BEARING_COORDS[0, 1]],
                            [LL_BEARING_COORDS[1, 0], LL_BEARING_COORDS[1, 1]],
                            antialias=True,
                        )
                        self.ll_bearings[id].setZValue(2)
                        self.plot.addItem(self.ll_bearings[id])

                    # Update ID
                    detected_ids.add(id)
                    self.ll_bearings[id].setPen(
                        BEARING_RETRO_PEN
                        if isRetro
                        else (
                            BEARING_TAG_PEN
                            if v.targetCornerID == -1
                            else BEARING_TAG_CORNER_PEN
                        )
                    )

                    tr = QtGui.QTransform()
                    tr.translate(x, y)
                    tr.rotateRadians(theta + v.bearing)
                    m = pg.transformToArray(tr)
                    coords = np.dot(m, LL_BEARING_COORDS)
                    self.ll_bearings[id].setData(
                        [coords[0, 0], coords[0, 1]], [coords[1, 0], coords[1, 1]]
                    )
            # Turn off all unseen ids
            for id, item in self.ll_bearings.items():
                if id not in detected_ids:
                    item.setPen(pg.mkPen(color=pg.mkColor(0, 0, 0, 0)))
