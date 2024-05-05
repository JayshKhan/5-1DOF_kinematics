# learning to plot

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider

# Define the number of joints
num_joints = 4

# Set up the base point for the first joint
base_x = 0
base_y = 0
base_z = 0

# Define the length of each link
link_length = 1

# Define initial angles for each joint (in degrees)
initial_angles = [45, 60, 60, 60]

# Create the plot figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Function to update plot based on slider values
def update(val):
    global base_x, base_y, base_z  # Declare base_x, base_y, and base_z as global

    ax.clear()
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    angles = [s.val for s in sliders]

    # Rotation about base Z for the first joint
    x_end = base_x + link_length * np.cos(np.radians(angles[0]))
    y_end = base_y + link_length * np.sin(np.radians(angles[0]))
    z_end = base_z
    ax.plot([base_x, x_end], [base_y, y_end], [base_z, z_end], '-o', linewidth=2, markersize=5, color='blue')

    # Update the base point for subsequent joints
    base_x = x_end
    base_y = y_end
    base_z = z_end

    # Rotation about base X for the remaining joints
    for i in range(1, num_joints):
        x_end = base_x
        y_end = base_y + link_length * np.sin(np.radians(angles[i]))
        z_end = base_z + link_length * np.cos(np.radians(angles[i]))
        ax.plot([base_x, x_end], [base_y, y_end], [base_z, z_end], '-o', linewidth=2, markersize=5, color='blue')
        base_x = x_end
        base_y = y_end
        base_z = z_end

    fig.canvas.draw_idle()


sliders = []
for i in range(num_joints):
    axslider = plt.axes([0.1, 0.95 - i * 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(axslider, f'Joint {i + 1}', 0, 180, valinit=initial_angles[i])
    slider.on_changed(update)
    sliders.append(slider)

update(0)  # Update plot with initial values

plt.show()
