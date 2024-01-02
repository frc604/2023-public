import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np

from utils.field import Field
from utils.robot import Robot
from utils.quikplan import QuikPlan, StoppedPoseConstraint, PoseConstraint
from utils.helpers import write_to_csv


def plan(quiet):
    # Create the field
    field = Field()

    # Create the robot model
    robot = Robot()

    # Configure the optimizer
    start_pose = (0, 0, 0)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        apply_boundaries=False,
    )
    waypoint2 = (1.0, 0.0, np.pi)
    qp.add_waypoint(waypoint2, 50, end_constraints=[StoppedPoseConstraint(waypoint2)])

    # Plan the trajectory
    traj = qp.plan()
    write_to_csv(traj, "simple_test")

    # Plot
    field.plot_traj(robot, traj, "simple_test.png", save=True, quiet=quiet)

    if not quiet:
        # Animate
        field.anim_traj(robot, traj, "simple_test.gif", save_gif=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(sys.argv[1:])
    plan(args.quiet)
