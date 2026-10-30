#!/usr/bin/env python3

import numpy as np
import typing as T

import rclpy
from scipy.interpolate import splev, splrep

from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw
from asl_tb3_lib.grids import StochOccupancyGrid2D
from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl


class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1,
                 heuristic_type="l2", heuristic_weight=1.0):
        self.statespace_lo = statespace_lo          # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi          # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                  # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution                # resolution of the discretization of state space (cell/m)
        self.heuristic_type = heuristic_type        # "l1", "l2" or "linf"
        self.heuristic_weight = heuristic_weight    # >1 makes the search greedy / inadmissible
        self.x_offset = x_init
        self.x_init = self.snap_to_grid(x_init)     # initial state
        self.x_goal = self.snap_to_grid(x_goal)     # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.heuristic(self.x_init)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """Checks if a give state x is free, meaning it is inside the bounds of
        the map and is not inside any obstacle."""
        blocked = (((x[0] < self.statespace_lo[0] or x[1] < self.statespace_lo[1]))
                   or (x[0] > self.statespace_hi[0] or x[1] > self.statespace_hi[1])
                   or self.occupancy.is_free(x) == False)
        return not blocked

    def distance(self, x1, x2):
        """Computes the Euclidean distance between two states."""
        return np.linalg.norm(np.subtract(np.array(x1), np.array(x2)))

    def heuristic(self, x):
        """Estimated cost-to-go from x to the goal."""
        difference = tuple(np.array(x) - np.array(self.x_goal))
        if self.heuristic_type == "l1":
            norm = np.linalg.norm(difference, ord=1)
        elif self.heuristic_type == "l2":
            norm = np.linalg.norm(difference, ord=2)
        elif self.heuristic_type == "linf":
            norm = np.linalg.norm(difference, ord=np.inf)
        else:
            raise ValueError("heuristic not recognized")
        return norm * self.heuristic_weight

    def snap_to_grid(self, x):
        """Returns the closest point on a discrete state grid."""
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """Gets the FREE neighbor states of a given state x (8-connected grid)."""
        neighbors = []
        offset = [(self.resolution, 0), (0, self.resolution),
                  (self.resolution, self.resolution), (-self.resolution, 0),
                  (-self.resolution, -self.resolution), (self.resolution, -self.resolution),
                  (0, -self.resolution), (-self.resolution, self.resolution)]
        for delta in offset:
            pos = self.snap_to_grid(np.array(delta) + np.array(x))
            if self.is_free(pos):
                neighbors.append(pos)
        return neighbors

    def find_best_est_cost_through(self):
        """Gets the state in open_set that has the lowest est_cost_through."""
        return min(self.open_set, key=lambda x: self.est_cost_through[x])

    def reconstruct_path(self):
        """Use the came_from map to reconstruct a path from the initial location
        to the goal location."""
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    def solve(self):
        """Solves the planning problem using the A* search algorithm.

        Returns:
            bool: True if a path is found, False otherwise. On success the path
                  is stored in self.path.
        """
        while len(self.open_set) > 0:
            x_current = self.find_best_est_cost_through()

            if x_current == self.x_goal:
                self.path = self.reconstruct_path()
                return True

            self.open_set.remove(x_current)
            self.closed_set.add(x_current)

            for x_neigh in self.get_neighbors(x_current):
                if x_neigh in self.closed_set:
                    continue
                tent_cost_to_arrive = self.distance(x_current, x_neigh) + self.cost_to_arrive[x_current]
                if x_neigh not in self.open_set:
                    self.open_set.add(x_neigh)
                else:
                    if tent_cost_to_arrive >= self.cost_to_arrive[x_neigh]:
                        continue
                self.came_from[x_neigh] = x_current
                self.cost_to_arrive[x_neigh] = tent_cost_to_arrive
                self.est_cost_through[x_neigh] = tent_cost_to_arrive + self.heuristic(x_neigh)

        return False


class Navigator(BaseNavigator):
    """Navigation node: A* path planning + cubic spline smoothing + differential
    flatness trajectory tracking."""

    def __init__(self) -> None:
        super().__init__("navigator")

        # heading controller gain (NavMode.ALIGN and NavMode.PARK)
        self.kp = 2.0

        # trajectory tracking gains (migrated from trajectory_tracking.py)
        self.kpx = 2.0
        self.kpy = 2.0
        self.kdx = 2.0
        self.kdy = 2.0

        # tracking controller ODE integration state
        self.V_PREV_THRESH = 0.0001
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

        # trajectory generation parameters
        self.v_desired = 0.15       # constant-velocity heuristic for time stamping [m/s]
        self.spline_alpha = 0.05    # smoothing factor passed to splrep

    def compute_heading_control(self,
        state: TurtleBotState,
        goal: TurtleBotState,
    ) -> TurtleBotControl:
        """Proportional control on heading error only (zero linear velocity)."""
        err = wrap_angle(goal.theta - state.theta)
        return TurtleBotControl(omega=self.kp * err)

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float,
    ) -> TurtleBotControl:
        """Differential flatness tracking controller.

        Virtual control (u1, u2) is a PD law on the flat outputs (x, y); it is
        then mapped back to (V, om) through the unicycle flatness relations,
        where V is obtained by integrating the acceleration a = u . [cos, sin].
        """
        dt = t - self.t_prev

        # desired flat outputs and their derivatives, sampled from the splines
        x_d   = float(splev(t, plan.path_x_spline, der=0))
        xd_d  = float(splev(t, plan.path_x_spline, der=1))
        xdd_d = float(splev(t, plan.path_x_spline, der=2))
        y_d   = float(splev(t, plan.path_y_spline, der=0))
        yd_d  = float(splev(t, plan.path_y_spline, der=1))
        ydd_d = float(splev(t, plan.path_y_spline, der=2))

        x, y, th = state.x, state.y, state.theta

        # guard against the singularity at V = 0 (om = ... / V)
        V_prev = self.V_prev
        if abs(V_prev) < self.V_PREV_THRESH:
            V_prev = self.V_PREV_THRESH

        # current velocity in flat space, assuming nonholonomic constraint
        xd = V_prev * np.cos(th)
        yd = V_prev * np.sin(th)

        # virtual control = feedforward acceleration + PD feedback
        u1 = xdd_d + self.kpx * (x_d - x) + self.kdx * (xd_d - xd)
        u2 = ydd_d + self.kpy * (y_d - y) + self.kdy * (yd_d - yd)

        # invert the flatness map
        a = u1 * np.cos(th) + u2 * np.sin(th)       # dV/dt
        V = V_prev + a * dt
        if abs(V) < self.V_PREV_THRESH:
            V = self.V_PREV_THRESH
        om = (u2 * np.cos(th) - u1 * np.sin(th)) / V

        # save for the next integration step
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        return TurtleBotControl(v=V, omega=om)

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float,
    ) -> T.Optional[TrajectoryPlan]:
        """A* over the occupancy grid, then cubic spline fit in time."""
        # 1. set up and solve the A* problem
        astar = AStar(
            statespace_lo=(state.x - horizon, state.y - horizon),
            statespace_hi=(state.x + horizon, state.y + horizon),
            x_init=(state.x, state.y),
            x_goal=(goal.x, goal.y),
            occupancy=occupancy,
            resolution=resolution,
        )

        # 2. a cubic spline needs at least 4 knots
        if not astar.solve() or len(astar.path) < 4:
            return None

        # 3. reset the tracking controller's integration history for the new plan
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

        path = np.asarray(astar.path)

        # 4. time stamps from a constant-velocity heuristic
        difference = np.diff(path, axis=0)
        distances = np.linalg.norm(difference, axis=1)
        time = distances / self.v_desired
        ts = np.cumsum(time)
        ts = np.concatenate(([0], ts))

        # 5. fit cubic splines to x(t) and y(t)
        path_x_spline = splrep(ts, path[:, 0], s=self.spline_alpha)
        path_y_spline = splrep(ts, path[:, 1], s=self.spline_alpha)

        return TrajectoryPlan(
            path=path,
            path_x_spline=path_x_spline,
            path_y_spline=path_y_spline,
            duration=ts[-1],
        )


if __name__ == "__main__":
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()
