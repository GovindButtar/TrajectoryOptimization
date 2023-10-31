import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import json
import math
import os
import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.transforms as transforms

def draw_field():
    fig, ax = plt.subplots()
    ax.add_patch(mpl.patches.Rectangle(
        (0, 0),
        30,
        15,
        lw=4,
        edgecolor="black",
        facecolor="none",
    ))
    plt.title("Trajectory")
    plt.xlabel("X Position (meters)")
    plt.ylabel("y Position (meters)")
    plt.gca().set_aspect("equal", adjustable="box")
    return fig, ax

def draw_trajectory(x_coords, y_coords, angular_coords, waypoints, drive, title):
    fig, ax = draw_field()

    plt.plot(x_coords,y_coords,color="b")
    plt.title(title)

def animate_trajectory(
    x_coords,
    y_coords,
    angular_coords,
    waypoints,
    drive,
    dt,
    title
):
    
    fig, ax = draw_field()

    num_states = len(x_coords)
    plt.plot(x_coords, y_coords)

    myrect = patches.Rectangle(
        (0, 0),
        drive.wheelbase_x, drive.wheelbase_y,
        fc="y",
        rotation_point="center")
    
    myrect.set_fill(False)
    

    def init():
        ax.add_patch(myrect)
        return myrect,

    def animate(i):

        robot_transform = transforms.Affine2D().translate(-drive.wheelbase_x / 2, -drive.wheelbase_y / 2).rotate(angular_coords[i]).translate(x_coords[i], y_coords[i]) + ax.transData 
        myrect.set_transform(robot_transform)
        myrect.set_x(0)
        myrect.set_y(0)
        myrect.set_angle(0)
        return myrect,

    anim = animation.FuncAnimation(
        fig, animate, init_func=init, frames=num_states, interval=40, blit=True, repeat=True
    )

    plt.show()

    if not os.path.exists("animations"):
        os.makedirs("animations")
    anim.save(
        os.path.join("animations", "{}.gif".format(title)),
        fps=(int)(1 / dt),
    )
    return anim