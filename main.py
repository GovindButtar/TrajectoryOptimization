from generator import Generator, ListedTrajectory
from swervedrive import swervedrive
import vizualization
import matplotlib.pyplot as plt
from math import pi

def main():
    drive = swervedrive(
      # Wheelbase x/y
        0.622,0.572,
        # Bumper length/width
        0.954,0.903,
        # Mass/moi
        46.7,5.6,
        # Max velocity/force
        70, 1.9,
        # 73, 1,
        # 50, 1,
        # Wheel radius
        0.0508)
    
    waypoints = [
        (0.0, 0.0, 0.0),
        (5, 4, pi/2),
        (5.0, 10, -pi/3)]

    generator = Generator(drive)

    trajectory = generator.generate(waypoints)

    vizualization.animate_trajectory(trajectory.x, trajectory.y, trajectory.theta, waypoints, drive, 0.02, "trajopt")
main()