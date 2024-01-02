import csv
import numpy as np
import os


class PoseConstructor:
    def __init__(self, field, alliance):
        self.field = field
        self.is_blue = alliance == "blue"

    def construct_pose(self, xs, ys, thetas):
        idx = 0 if self.is_blue else 1
        x = xs[idx] if type(xs) is tuple else xs
        y = ys[idx] if type(ys) is tuple else ys
        theta = thetas[idx] if type(thetas) is tuple else thetas

        if self.is_blue:
            return (x, y, theta)
        else:
            return (self.field.LENGTH - x, y, theta)


def in2m(inches):
    # Inches to meters
    return inches * 2.54 / 100.0


def rotate_around_origin(point, theta):
    x, y = point
    return (
        x * np.cos(theta) - y * np.sin(theta),
        y * np.cos(theta) + x * np.sin(theta),
    )


def transform_geometry(geometry, pose):
    x, y, theta = pose
    transformed_geometry = []
    for point1, point2 in geometry:
        new_point1 = rotate_around_origin(point1, theta) + np.array([x, y])
        new_point2 = rotate_around_origin(point2, theta) + np.array([x, y])
        transformed_geometry.append((new_point1, new_point2))
    return transformed_geometry


def interp_state_vector(times, states, new_dt):
    interp_times = np.arange(0, times[-1], new_dt)
    return interp_times, np.interp(interp_times, times, states)


def interp_actions_vector(times, actions, new_dt, null_action):
    interp_times = np.arange(0, times[-1], new_dt)
    interp_action_type = []
    interp_grid_id = []
    interp_node_id = []
    action_idx = 0
    for time in interp_times:
        if action_idx < len(actions) and time >= times[action_idx]:
            interp_action_type.append(int(actions[action_idx].action_type))
            scoring_loc = actions[action_idx].scoring_loc
            if scoring_loc is not None:
                interp_grid_id.append(scoring_loc[0])
                interp_node_id.append(scoring_loc[1])
            else:
                interp_grid_id.append(-1)
                interp_node_id.append(-1)
            action_idx += 1
        else:
            interp_action_type.append(int(null_action.action_type))
            interp_grid_id.append(-1)
            interp_node_id.append(-1)

    # Sometimes interp doesnt reach the end time.
    if time < times[-1]:
        interp_action_type[-1] = int(actions[-1].action_type)
        scoring_loc = actions[-1].scoring_loc
        if scoring_loc is not None:
            interp_grid_id[-1] = scoring_loc[0]
            interp_node_id[-1] = scoring_loc[1]
        else:
            interp_grid_id[-1] = -1
            interp_node_id[-1] = -1

    return interp_action_type, interp_grid_id, interp_node_id


def write_to_csv(traj, name):
    with open(
        os.path.join(
            os.path.dirname(__file__), "../../../src/main/deploy/{}.csv".format(name)
        ),
        "w",
        newline="",
    ) as outfile:
        writer = csv.writer(outfile, delimiter=",")
        writer.writerows(traj)
    outfile.close()
