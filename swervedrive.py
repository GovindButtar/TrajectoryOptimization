from math import hypot, pi
from casadi import *

class swervedrive:
    def __init__(self, wheelbase_x: float, wheelbase_y: float, length: float, width: float,
            mass: float, moi: float, omega_max: float, tau_max: float, wheel_radius: float):
        """
                wheelbase_x  -- half the distance between modules
                wheelbase_y  -- half the distance between modules
                length       -- distance in x direction
                width        -- distance in y direction
                mass         -- mass of robot
                moi          -- moment of inertia of robot
                omega_max    -- maximum angular velocity of wheels
                tau_max      -- maximum torque of wheels
                wheel_radius -- radius of wheels
        """
        self.wheelbase_x = wheelbase_x
        self.wheelbase_y = wheelbase_y
        self.length = length
        self.width = width
        self.mass = mass
        self.moi = moi
        self.omega_max = omega_max
        self.tau_max = tau_max
        self.wheel_radius = wheel_radius
        self.force_max = tau_max / wheel_radius

    def solve_module_positions(self, theta):
        module_angle = atan2(self.wheelbase_y, self.wheelbase_x)
        module_angles = (module_angle, -module_angle, pi - module_angle, -(pi - module_angle))
        diagonal = hypot(self.wheelbase_x, self.wheelbase_y)
        module_positions = []
        for module_angle in module_angles:
            module_positions.append([diagonal*cos(module_angle+theta), diagonal*sin(module_angle+theta)])
        return module_positions

    def add_kinematics_constraint(self, solver, theta, vx, vy, omega, ax, ay, alpha, N_total):

        max_wheel_velocity = self.omega_max * self.wheel_radius
        for k in range(N_total):
            module_positions = self.solve_module_positions(theta[k])

            for module_position in module_positions:

                m_vx = vx[k] + module_position[0] * omega[k]
                m_vy = vy[k] + module_position[1] * omega[k]
                solver.subject_to((m_vx * m_vx + m_vy * m_vy) < max_wheel_velocity * max_wheel_velocity)

            Fx = solver.variable(4)
            Fy = solver.variable(4)

            T = []
            for j in range(4):
                T.append(module_positions[j][1] * Fx[j] - module_positions[j][0] * Fy[j])
                solver.subject_to(Fx[j] * Fx[j] + Fy[j] * Fy[j] < self.force_max * self.force_max)

            solver.subject_to(ax[k] * self.mass == Fx[0] + Fx[1] + Fx[2] + Fx[3])
            solver.subject_to(ay[k] * self.mass == Fy[0] + Fy[1] + Fy[2] + Fy[3])
            solver.subject_to(alpha[k] * self.moi == sum(T))