import casadi as ca
from enum import IntEnum
import numpy as np
import matplotlib.pyplot as plt

from utils.helpers import in2m, interp_state_vector, interp_actions_vector


class BasePoseConstraint(object):
    def __init__(self, pose):
        self.pose = pose


class StoppedPoseConstraint(BasePoseConstraint):
    pass


class PoseConstraint(BasePoseConstraint):
    pass


class XConstraint(BasePoseConstraint):
    pass


class YConstraint(BasePoseConstraint):
    pass


class XYConstraint(BasePoseConstraint):
    pass


class StoppedXYConstraint(BasePoseConstraint):
    pass


class AngularConstraint(BasePoseConstraint):
    pass


class StayStoppedConstraint(object):
    def __init__(self, time):
        self.time = time


# Hacky way to enforce stop time
class SpeedConstraint(object):
    def __init__(self, speed):
        self.speed = speed


class InitializationConstraint(object):
    pass


class ActionType(IntEnum):
    NONE = 0
    SCORE_PIECE = 1
    DEPLOY_INTAKE = 2
    RETRACT_INTAKE = 3
    BALANCE = 4
    BALANCE_REVERSE = 5
    POOP_CHUTE = 6
    DRIVE_OFF_CHARGING_STATION_AND_BALANCE = 7


class Action(object):
    def __init__(self, action_type=ActionType.NONE, scoring_loc=None):
        self.action_type = action_type
        self.scoring_loc = scoring_loc


class QuikPlan(object):
    BALL_VELOCITY = 8.0  # m/s

    def __init__(
        self,
        field,
        robot,
        start_pose,
        start_constraints,
        start_action=Action(),
        apply_boundaries=True,
    ):
        self._field = field
        self._robot = robot
        self._apply_boundaries = apply_boundaries

        # Keep track of states as an initial guess
        self._states = np.zeros((1, 3))
        self._states[0, :] = start_pose

        # Dict of state_idx: action
        self._actions = {0: start_action}

        # List of the number of control intervals between each waypoint. There are |sum(Ns) + 1| states.
        self._Ns = []

        # List of (state_idx, constraint) tuples
        self._waypoint_idx = -1
        self._constraints = []
        for c in start_constraints:
            self._constraints.append((0, c, self._waypoint_idx))

    def add_waypoint(
        self,
        pose,
        N,
        intermediate_constraints=[],
        end_constraints=[],
        end_action=Action(),
    ):
        # Linearly interpolate to initialize states
        # TODO: Fix excessive copying
        last_state = self._states[-1, :]
        new_states = np.zeros((N, 3))
        new_states[:, 0] = np.linspace(last_state[0], pose[0], N + 1)[1:]
        new_states[:, 1] = np.linspace(last_state[1], pose[1], N + 1)[1:]
        new_states[:, 2] = np.linspace(last_state[2], pose[2], N + 1)[1:]
        self._states = np.vstack((self._states, new_states))

        # Store end action
        self._actions[len(self._states) - 1] = end_action

        # Save constraints
        self._waypoint_idx += 1
        start_N = sum(self._Ns)
        for i in range(start_N, start_N + N + 1):
            for c in intermediate_constraints:
                self._constraints.append((i, c, self._waypoint_idx))
            if i == start_N + N:
                for c in end_constraints:
                    self._constraints.append((i, c, self._waypoint_idx))

        # Update Ns
        self._Ns.append(N)

    def plot_init(self, mod=10):
        fig, ax = plt.subplots()
        self._field.plot_field(ax)

        num_states = self._states.shape[0]
        for i in range(num_states):
            state = self._states[i]
            self._robot.plot(ax, state)
        plt.show()

    def plan(self):
        # Construct optimization problem
        opti = ca.Opti()
        N = sum(self._Ns)

        # State variables
        X = opti.variable(9, N + 1)
        xpos = X[0, :]
        ypos = X[1, :]
        theta = X[2, :]
        xDot = X[3, :]
        yDot = X[4, :]
        thetaDot = X[5, :]
        xDotDot = X[6, :]
        yDotDot = X[7, :]
        thetaDotDot = X[8, :]

        # Control variables
        U = opti.variable(3, N)
        xDotDotDot = U[0, :]
        yDotDotDot = U[1, :]
        thetaDotDotDot = U[2, :]

        # Total time variable per segment
        Ts = []
        dts = []
        for n in self._Ns:
            T = opti.variable()
            dt = T / n
            Ts.append(T)
            dts.append(dt)

            # Apply time constraint & initial guess
            opti.subject_to(T >= 0)
            opti.set_initial(T, 5)

        # Minimize time + control^2, with much lower weight on control
        # Penalizing control is necessary because swerve has overconstrained DOFs,
        # which can result in unnecessary motion that doesn't add to the total
        # trajectory time. By penalizing control, we reduce this excess motion.
        LINEAR_CONTROL_WEIGHT = 1e-5
        ANGULAR_CONTROL_WEIGHT = 1e-4
        control_cost = 0.0
        # for n in range(N):
        #     control_cost += LINEAR_CONTROL_WEIGHT * xDotDotDot[n] * xDotDotDot[n]
        #     control_cost += LINEAR_CONTROL_WEIGHT * yDotDotDot[n] * yDotDotDot[n]
        #     control_cost += ANGULAR_CONTROL_WEIGHT * thetaDotDotDot[n] * thetaDotDotDot[n]
        total_time = sum(Ts)
        opti.minimize(total_time + control_cost)

        # Apply dynamic constriants
        start_n = 0
        for n, dt in zip(self._Ns, dts):
            end_n = start_n + n
            for k in range(start_n, end_n):
                x_next = X[:, k] + self._robot.dynamics_model(X[:, k], U[:, k]) * dt
                opti.subject_to(X[:, k + 1] == x_next)
            start_n = end_n

        # Apply whole robot limits
        LINEAR_VEL_LIMIT = 3.0  # m/s
        LINEAR_VEL_LIMIT_SQ = LINEAR_VEL_LIMIT * LINEAR_VEL_LIMIT
        ROTATIONAL_VEL_LIMIT = 2.0 * np.pi  # rad/s
        opti.subject_to(
            opti.bounded(
                -LINEAR_VEL_LIMIT_SQ, xDot * xDot + yDot * yDot, LINEAR_VEL_LIMIT_SQ
            )
        )
        opti.subject_to(
            opti.bounded(-ROTATIONAL_VEL_LIMIT, thetaDot, ROTATIONAL_VEL_LIMIT)
        )

        LINEAR_ACCEL_LIMIT = 2.0  # m/s/s
        LINEAR_ACCEL_LIMIT_SQ = LINEAR_ACCEL_LIMIT * LINEAR_ACCEL_LIMIT
        ROTATIONAL_ACCEL_LIMIT = 1.0 * np.pi  # rad/s/s
        opti.subject_to(
            opti.bounded(
                -LINEAR_ACCEL_LIMIT_SQ,
                xDotDot * xDotDot + yDotDot * yDotDot,
                LINEAR_ACCEL_LIMIT_SQ,
            )
        )
        opti.subject_to(
            opti.bounded(-ROTATIONAL_ACCEL_LIMIT, thetaDotDot, ROTATIONAL_ACCEL_LIMIT)
        )

        LINEAR_JERK_LIMIT = 9.0  # m/s/s/s
        LINEAR_JERK_LIMIT_SQ = LINEAR_JERK_LIMIT * LINEAR_JERK_LIMIT
        ROTATIONAL_JERK_LIMIT = 6.0 * np.pi  # rad/s/s/s
        opti.subject_to(
            opti.bounded(
                -LINEAR_JERK_LIMIT_SQ,
                xDotDotDot * xDotDotDot + yDotDotDot * yDotDotDot,
                LINEAR_JERK_LIMIT_SQ,
            )
        )
        opti.subject_to(
            opti.bounded(-ROTATIONAL_JERK_LIMIT, thetaDotDotDot, ROTATIONAL_JERK_LIMIT)
        )

        if self._apply_boundaries:
            # Apply field boundary constraints
            opti.subject_to(
                opti.bounded(
                    self._robot.WIDTH * 0.5,
                    xpos,
                    self._field.LENGTH - self._robot.WIDTH * 0.5,
                )
            )
            opti.subject_to(
                opti.bounded(
                    self._robot.WIDTH * 0.5,
                    ypos,
                    self._field.WIDTH - self._robot.WIDTH * 0.5,
                )
            )

        # Apply state constraints
        for i, constraint, waypoint_idx in self._constraints:
            if type(constraint) in {StoppedPoseConstraint, PoseConstraint}:
                opti.subject_to(X[0, i] == constraint.pose[0])
                opti.subject_to(X[1, i] == constraint.pose[1])
            if type(constraint) in {
                StoppedPoseConstraint,
                PoseConstraint,
                AngularConstraint,
            }:
                opti.subject_to(X[2, i] == constraint.pose[2])
            if type(constraint) in {StoppedPoseConstraint, StoppedXYConstraint}:
                opti.subject_to(X[3, i] == 0.0)
                opti.subject_to(X[4, i] == 0.0)
                opti.subject_to(X[5, i] == 0.0)
            if type(constraint) in {XConstraint, XYConstraint, StoppedXYConstraint}:
                opti.subject_to(X[0, i] == constraint.pose[0])
            if type(constraint) in {YConstraint, XYConstraint, StoppedXYConstraint}:
                opti.subject_to(X[1, i] == constraint.pose[1])
            if type(constraint) == SpeedConstraint:
                self._robot.apply_speed_constraint(opti, X, U, i, constraint.speed)
            if type(constraint) == StayStoppedConstraint:
                if i > 0:
                    opti.subject_to(X[0:3, i] == X[0:3, i - 1])
                    opti.subject_to(Ts[waypoint_idx] == constraint.time)
            if type(constraint) == InitializationConstraint:
                opti.subject_to(Ts[waypoint_idx] == 0.0)

        # Apply module torque/friction constraints
        # v0s, v1s, v2s, v3s, f0s, f1s, f2s, f3s = self._robot.apply_module_constraints(
        #     opti, X, U, N
        # )

        # Apply obstacle constraints
        self._robot.apply_obstacle_constraints(
            opti, xpos, ypos, theta, self._field.OBSTACLES
        )

        # Set initial guess
        opti.set_initial(xpos, self._states[:, 0])
        opti.set_initial(ypos, self._states[:, 1])
        opti.set_initial(theta, self._states[:, 2])

        # Solve
        opti.solver("ipopt")
        sol = opti.solve()
        for t in Ts:
            print(sol.value(t))
        print(f"Total time: {sol.value(total_time)}")

        # Interpolate result
        times = [0.0]
        action_idx = 0
        start_action = self._actions.get(0, Action())
        actions = [start_action]
        for n, t, dt in zip(self._Ns, Ts, dts):
            times += list(
                np.linspace(times[-1] + sol.value(dt), times[-1] + sol.value(t), n)
            )
            actions += (n - 1) * [Action()]

            action_idx += n
            action = self._actions.get(action_idx, Action())
            actions.append(action)

        interp_times, interp_x = interp_state_vector(times, sol.value(xpos), 0.02)
        _, interp_y = interp_state_vector(times, sol.value(ypos), 0.02)
        _, interp_theta = interp_state_vector(times, sol.value(theta), 0.02)
        _, interp_xDot = interp_state_vector(times, sol.value(xDot), 0.02)
        _, interp_yDot = interp_state_vector(times, sol.value(yDot), 0.02)
        _, interp_thetaDot = interp_state_vector(times, sol.value(thetaDot), 0.02)
        interp_action_type, interp_grid_id, interp_node_id = interp_actions_vector(
            times, actions, 0.02, Action()
        )

        # # Plot velocities/forces
        # plt.figure(figsize=(4.6, 4))
        # _, interp_v0s = interp_state_vector(times, [sol.value(v) for v in v0s], 0.02)
        # _, interp_v1s = interp_state_vector(times, [sol.value(v) for v in v1s], 0.02)
        # _, interp_v2s = interp_state_vector(times, [sol.value(v) for v in v2s], 0.02)
        # _, interp_v3s = interp_state_vector(times, [sol.value(v) for v in v3s], 0.02)
        # plt.plot(interp_times, interp_v0s, label="Module 0")
        # plt.plot(interp_times, interp_v1s, label="Module 1")
        # plt.plot(interp_times, interp_v2s, label="Module 2")
        # plt.plot(interp_times, interp_v3s, label="Module 3")
        # plt.legend(loc="upper left")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Velocity (m/s)")

        # plt.figure(figsize=(4.6, 4))
        # _, interp_f0s = interp_state_vector(times, [sol.value(f) for f in f0s], 0.02)
        # _, interp_f1s = interp_state_vector(times, [sol.value(f) for f in f1s], 0.02)
        # _, interp_f2s = interp_state_vector(times, [sol.value(f) for f in f2s], 0.02)
        # _, interp_f3s = interp_state_vector(times, [sol.value(f) for f in f3s], 0.02)
        # plt.plot(interp_times, interp_f0s, label="Module 0")
        # plt.plot(interp_times, interp_f1s, label="Module 1")
        # plt.plot(interp_times, interp_f2s, label="Module 2")
        # plt.plot(interp_times, interp_f3s, label="Module 3")
        # plt.legend(loc="upper left")
        # plt.xlabel("Time (s)")
        # plt.ylabel("Force (N)")

        return np.transpose(
            np.vstack(
                [
                    interp_times,
                    interp_x,
                    interp_y,
                    interp_theta,
                    interp_xDot,
                    interp_yDot,
                    interp_thetaDot,
                    interp_action_type,
                    interp_grid_id,
                    interp_node_id,
                ]
            )
        )
