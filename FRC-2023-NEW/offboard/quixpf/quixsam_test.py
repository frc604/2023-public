from turtle import position
import matplotlib.pyplot as plt
from multiprocessing import Process
import ntcore as nt
import numpy as np
import os
import time
import transformations as tf

from helpers import in2m, cart2besph, get_point_in_frame, plot_field, load_traj
from particle_filter import CAMERAS_TRANSFORMS, get_point_on_target_circle
from quixsam import Quixsam


def mock_robot():
    inst = nt.NetworkTableInstance.getDefault()
    inst.startServer()

    quixsam_table = inst.getTable("localizer")
    targets_pub = quixsam_table.getDoubleArrayTopic("targets").publish(
        nt.PubSubOptions(sendAll=True)
    )
    measurements_pub = quixsam_table.getDoubleArrayTopic("measurements").publish(
        nt.PubSubOptions(sendAll=True)
    )
    estimates_sub = quixsam_table.getDoubleArrayTopic("estimates").subscribe(
        [], nt.PubSubOptions(sendAll=True)
    )

    # Publish target
    TARGET_X = 1.0  # m
    TARGET_Y = 1.0  # m
    TARGET_Z = 0.0  # m
    targets_pub.set([-1, TARGET_X, TARGET_Y, TARGET_Z, 0, 0, 0, 0])

    HALF_FIELD_X = in2m(12 * 54 + 3.25) * 0.5  # m
    HALF_FIELD_Y = in2m(12 * 26 + 3.5) * 0.5  # m

    # Store estimates
    xs = []
    ys = []

    traj = load_traj(
        os.path.join(os.path.dirname(__file__), "./trajectories/orbit.csv")
    )
    for id in range(traj.shape[0]):
        # Mock odometry
        x, y, theta = traj[id, 1:4]
        # Shift by half field because orbit.csv was designed with 0, 0 at the center of the field.
        x += HALF_FIELD_X
        y += HALF_FIELD_Y
        pub_data = [id, x, y, theta, 0, 0, 0]

        # Mock vision
        target_direction_mod = np.mod(np.arctan2(y, x) + np.pi, 2 * np.pi)
        theta_mod = np.mod(theta, 2.0 * np.pi)

        # Check that we are facing the target within some FOV
        vision_array = []
        if np.pi - abs(abs(target_direction_mod - theta_mod) - np.pi) < np.deg2rad(45):
            T = tf.compose_matrix(translate=[x, y, 0], angles=[0, 0, theta])
            robot_to_camera_T = T.dot(CAMERAS_TRANSFORMS[0])
            mx, my, mz = get_point_in_frame(
                robot_to_camera_T, np.array([[TARGET_X], [TARGET_Y], [TARGET_Z]])
            )
            be, ev = cart2besph(mx, my, mz)
            vision_array = [
                0,  # Camera ID
                -1,  # Target ID
                -1,  # Fiducial corner ID
                be,
                ev,
                0,
                0,
            ]

        pub_data.extend(vision_array)
        measurements_pub.set(pub_data)
        inst.flush()
        time.sleep(0.02)

    # Receive values
    for _, x, y, _ in estimates_sub.readQueue():
        xs.append(x)
        ys.append(y)

    # Plot result vs. ground truth
    _, ax = plt.subplots()
    plt.axis("equal")
    plot_field(ax)
    plt.plot(traj[:, 1], traj[:, 2])  # Ground truth
    plt.scatter(xs, ys, marker=".", color="r")  # Estimate
    plt.show()


def run_quixsam():
    Quixsam("localhost", save_logs=False, view3d=True).run()


if __name__ == "__main__":
    # Start Quixsam
    qs = Process(target=run_quixsam)
    qs.start()

    # Start running mock robot
    p = Process(target=mock_robot)
    p.start()

    # Join threads
    qs.join()
    p.join()
