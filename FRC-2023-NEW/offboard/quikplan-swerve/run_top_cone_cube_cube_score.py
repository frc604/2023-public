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
    start_pose = pc.construct_pose(*TOP_START_POS)
    qp = QuikPlan(
        field,
        robot,
        start_pose,
        [StoppedPoseConstraint(start_pose)],
        start_action=Action(ActionType.SCORE_PIECE, scoring_loc=(0, 6)),
    )

    # Leave community without turning
    waypoint1 = pc.construct_pose(*TOP_COMMUNITY_EXIT)
    qp.add_waypoint(
        waypoint1,
        10,
        intermediate_constraints=[
            AngularConstraint(waypoint1),  # Maintain heading within the community
        ],
        end_constraints=[PoseConstraint(waypoint1)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    # Get topmost/second piece (cube)
    waypoint2 = pc.construct_pose(*TOP_FIRST_PIECE_APPROACH)
    qp.add_waypoint(
        waypoint2,
        10,
        end_constraints=[PoseConstraint(waypoint2)],
    )

    # Grab topmost/second cube at a slow speed
    waypoint3 = pc.construct_pose(*TOP_FIRST_PIECE)
    qp.add_waypoint(
        waypoint3,
        10,
        intermediate_constraints=[
            AngularConstraint(waypoint3),  # Maintain heading during pickup
        ],
        end_constraints=[StoppedPoseConstraint(waypoint3)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    # Prepare to enter community
    waypoint4 = pc.construct_pose(*TOP_COMMUNITY_ENTRANCE)
    qp.add_waypoint(
        waypoint4,
        10,
        end_constraints=[PoseConstraint(waypoint4)],
    )

    # This is used for initialization only.
    waypoint4andahalf = pc.construct_pose(*TOP_SCORING_INIT)
    qp.add_waypoint(
        waypoint4andahalf, 1, intermediate_constraints=[InitializationConstraint()]
    )

    # Go to score second/topmost piece (cube)
    waypoint5 = pc.construct_pose(*TOP_THIRD_SCORING_POS)
    qp.add_waypoint(
        waypoint5,
        10,
        intermediate_constraints=[
            AngularConstraint(waypoint5),  # Maintain heading within the community
        ],
        end_constraints=[
            StoppedPoseConstraint(waypoint5),  # We want to be stopped when scoring
        ],
        end_action=Action(ActionType.SCORE_PIECE, scoring_loc=(0, 7)),
    )

    # This is used for initialization only.
    waypoint5andahalf = pc.construct_pose(*TOP_SCORING_INIT)
    qp.add_waypoint(
        waypoint5andahalf, 1, intermediate_constraints=[InitializationConstraint()]
    )

    # Leave community without turning
    waypoint6 = pc.construct_pose(*TOP_COMMUNITY_ENTRANCE)
    qp.add_waypoint(
        waypoint6,
        10,
        intermediate_constraints=[
            AngularConstraint(waypoint6),  # Maintain heading within the community
        ],
        end_constraints=[PoseConstraint(waypoint6)],
        end_action=Action(ActionType.DEPLOY_INTAKE),
    )

    # Get near 3rd piece (cube)
    waypoint7 = pc.construct_pose(*TOP_SECOND_PIECE_APPROACH)
    qp.add_waypoint(
        waypoint7,
        10,
        end_constraints=[PoseConstraint(waypoint7)],
    )

    # Grab 3rd piece at slow speed (cube)
    waypoint8 = pc.construct_pose(*TOP_SECOND_PIECE)
    qp.add_waypoint(
        waypoint8,
        10,
        intermediate_constraints=[AngularConstraint(waypoint8)],
        end_constraints=[StoppedPoseConstraint(waypoint8)],
        end_action=Action(ActionType.RETRACT_INTAKE),
    )

    # This is used for initialization only.
    waypoint8andahalf = pc.construct_pose(*TOP_SECOND_PIECE_APPROACH)
    qp.add_waypoint(
        waypoint8andahalf, 1, intermediate_constraints=[InitializationConstraint()]
    )

    # Prepare to enter community
    waypoint9 = pc.construct_pose(*TOP_COMMUNITY_ENTRANCE)
    qp.add_waypoint(
        waypoint9,
        10,
        end_constraints=[PoseConstraint(waypoint9)],
    )

    # This is used for initialization only.
    waypoint9andahalf = pc.construct_pose(*TOP_SCORING_INIT)
    qp.add_waypoint(
        waypoint9andahalf, 1, intermediate_constraints=[InitializationConstraint()]
    )

    # Go to score third piece (cube, IN MID ROW)
    waypoint10 = pc.construct_pose(*TOP_THIRD_SCORING_POS)
    qp.add_waypoint(
        waypoint10,
        10,
        intermediate_constraints=[
            AngularConstraint(waypoint10),  # Maintain heading within the community
        ],
        end_constraints=[
            StoppedPoseConstraint(waypoint10),  # We want to be stopped when scoring
        ],
        end_action=Action(ActionType.SCORE_PIECE, scoring_loc=(0, 4)),
    )

    # Plan the trajectory
    if not quiet:
        qp.plot_init()

    traj = qp.plan()

    write_to_csv(traj, f"{alliance}_top_cone_cube_cube_score")

    # Plot
    field.plot_traj(
        robot, traj, f"{alliance}_top_cone_cube_cube_score.png", save=True, quiet=quiet
    )

    if not quiet:
        # Animate
        field.anim_traj(
            robot, traj, f"{alliance}_top_cone_cube_cube_score.gif", save_gif=False
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
