from dataclasses import dataclass


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
    id: int
    cameraID: int
    targetID: int
    targetCornerID: int
    bearing: float
    elevation: float


@dataclass
class Odometry:
    id: int
    x: float
    y: float
    theta: float
