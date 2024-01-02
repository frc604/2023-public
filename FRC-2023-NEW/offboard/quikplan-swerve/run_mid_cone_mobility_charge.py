import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np

from constants import *
from utils.field import Field
from utils.robot import Robot
from utils.helpers import in2m
from utils.quikplan import (
    QuikPlan,
    StoppedPoseConstraint,
    PoseConstraint,
    StoppedXYConstraint,
    XYConstraint,
    AngularConstraint,
    XConstraint,
    YConstraint,
    SpeedConstraint,
    InitializationConstraint,
    ActionType,
    Action,
)
from utils.helpers import PoseConstructor, write_to_csv


def plan(alliance, quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot()

    # Create the pose constructor
    pc = PoseConstructor(field, alliance)

    # Configure the optimizer
    start_pose = pc.construct_pose(*MID_START_POS)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        start_action=Action(ActionType.SCORE_PIECE, scoring_loc=(1, 8)),
    )

    # Back up barely
    waypoint1 = pc.construct_pose(*MID_CHARGE_STATION_INSIDE)
    qp.add_waypoint(
        waypoint1,
        10,
        end_constraints=[StoppedPoseConstraint(waypoint1)],
        end_action=Action(ActionType.DRIVE_OFF_CHARGING_STATION_AND_BALANCE),
    )

    # Plan the trajectory
    if not quiet:
        qp.plot_init()

    traj = qp.plan()

    write_to_csv(traj, f"{alliance}_mid_cone_mobility_charge")

    # Plot
    field.plot_traj(
        robot, traj, f"{alliance}_mid_cone_mobility_charge.png", save=True, quiet=quiet
    )

    if not quiet:
        # Animate
        field.anim_traj(
            robot, traj, f"{alliance}_mid_cone_mobility_charge.gif", save_gif=False
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "alliance",
        choices=["blue", "red"],
    )
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.alliance, args.quiet)
