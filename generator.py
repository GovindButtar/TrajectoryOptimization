from typing import List, Dict
import casadi
from casadi import *
from swervedrive import swervedrive
from dataclasses import dataclass
@dataclass 
class TrajectoryList():
    ts: list[float]
    x: list[float]
    y: list[float]
    theta: list[float]
    vx: list[float]
    vy: list[float]
    omega: list[float]

class Generator:
    def __init__(self, drive):
        self.drive = drive

    def generate(self, waypoints: List[Dict]):
        self.N_per_trajectory_segment = 100
        self.trajectory_segment_count = len(waypoints) - 1
        self.N_total = self.N_per_trajectory_segment * self.trajectory_segment_count

        self.opti = Opti()

        Ts, dts = [], []
        for k in range(self.trajectory_segment_count):
            T = self.opti.variable()
            dt = T / self.N_per_trajectory_segment
            Ts.append(T)
            dts.append(dt)

            self.opti.subject_to(T >= 0)
            self.opti.set_initial(T, 5)
        self.opti.minimize(sum(Ts))

        self.X = self.opti.variable(6, self.N_total+1)

        self.x = self.X[0,:]
        self.y = self.X[1,:]
        self.theta = self.X[2,:]
        self.vx = self.X[3,:]
        self.vy = self.X[4,:]
        self.omega = self.X[5,:]

        self.U = self.opti.variable(3, self.N_total)

        self.ax = self.U[0,:]
        self.ay = self.U[1,:]
        self.alpha = self.U[2,:]

        dynamics = lambda X, U: vertcat(
            X[3], # vx
            X[4], # vy
            X[5], # omega
            U[0], # ax
            U[1], # ay
            U[2]  # alpha
        )

        for k in range(self.N_total):
            x_next = self.X[:, k] + dynamics(self.X[:, k], self.U[:, k]) * dts[k // self.N_per_trajectory_segment]
            self.opti.subject_to(self.X[:, k + 1] == x_next)

        x_init, y_init, theta_init = initial_values(waypoints, self.N_per_trajectory_segment)
        self.opti.set_initial(self.x, x_init)
        self.opti.set_initial(self.y, y_init)
        self.opti.set_initial(self.theta, theta_init)

        self.drive.add_kinematics_constraint(self.opti, self.theta, self.vx, self.vy, self.omega, self.ax, self.ay, self.alpha, self.N_total)
        self.boundry_constraint()
        self.add_waypoint_constraint(waypoints)

        self.opti.solver("ipopt")
        solution = self.opti.solve()

        solution_dts = []
        for k in range(self.trajectory_segment_count):
            solution_dts.append(solution.value(Ts[k] / self.N_per_trajectory_segment)) 

        solution_x = solution.value(self.x)
        solution_y = solution.value(self.y)
        solution_theta = solution.value(self.theta)
        solution_vx = solution.value(self.vx)
        solution_vy = solution.value(self.vy)
        solution_omega = solution.value(self.omega)

        ts = [0]
        for solution_dt in solution_dts:
            for k in range(self.N_per_trajectory_segment):
                ts.append(ts[-1] + solution_dt)
        trajectory = []
        for j in range(len(solution_x)):
            trajectory.append({'ts': round(ts[j],4), 'x': round(solution_x[j],4), 'y': round(solution_y[j],4), 'heading': round(solution_theta[j],4), 'vx': round(solution_vx[j],4), 'vy': round(solution_vy[j],4), 'omega': round(solution_omega[j],4)})

        trajectory1 = TrajectoryList(
            ts,
            solution_x.tolist(),
            solution_y.tolist(),
            solution_theta.tolist(),
            solution_vx.tolist(),
            solution_vy.tolist(),
            solution_omega.tolist()
        )
        return trajectory1

    def boundry_constraint(self):
        for k in [-1, 0]:
            self.opti.subject_to(self.vx[k] == 0)
            self.opti.subject_to(self.vy[k] == 0)
            self.opti.subject_to(self.omega[k] == 0)

    def add_waypoint_constraint(self, waypoints: List[Dict]):
        for k in range(self.trajectory_segment_count + 1):
            index = k * self.N_per_trajectory_segment
            self.opti.subject_to(self.x[index] == waypoints[k][0])
            self.opti.subject_to(self.y[index] == waypoints[k][1])
            self.opti.subject_to(self.theta[index] == waypoints[k][2])

def initial_values(waypoints: List[Dict], N_per_trajectory_segment: int):
    x, y, theta = [], [], []
    for k in range(len(waypoints) - 1):
        x.extend(np.linspace(waypoints[k][0], waypoints[k+1][0], N_per_trajectory_segment, False).tolist())
        y.extend(np.linspace(waypoints[k][1], waypoints[k+1][1], N_per_trajectory_segment, False).tolist())
        theta.extend(np.linspace(waypoints[k][2], waypoints[k+1][2], N_per_trajectory_segment, False).tolist())
    x.append(waypoints[-1][0]) 
    y.append(waypoints[-1][1])
    theta.append(waypoints[-1][2])
    return x, y, theta