from dataclasses import dataclass
from typing import List


@dataclass
class Fiducial:
    id: int
    x: float
    y: float
    z: float
    xRot: float
    yRot: float
    zRot: float
    size: float


@dataclass
class Vision:
    cameraID: int
    targetID: int
    targetCornerID: int
    bearing: float
    elevation: float


@dataclass
class Odometry:
    x: float
    y: float
    theta: float


@dataclass
class Measurement:
    id: int
    odom: Odometry
    vision: List[Vision]
