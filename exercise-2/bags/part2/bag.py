import rosbag
import math
import numpy as np
import matplotlib.pyplot as plt

bag_location = "/home/roboticslab229l/Documents/ros-phamm/exercise-2/bags/part2/rotate.bag"
bag = rosbag.Bag(bag_location)
# print(bag.get_type_and_topic_info())

point_list = []
for topic, msg, t in bag.read_messages():
    # if msg.vel_left != 0 and msg.vel_right != 0:
    point_list.append((t, msg.vel_left, msg.vel_right))

# print(point_list)


# segment_list = []
# wheel_distance = 2 * math.pi * 0.0318
# for tuple1 in point_list:
#     left_distance = tuple1[1] * 0.05 / (2 * math.pi * 0.66)
#     right_distance = tuple1[2] * 0.05 / (2 * math.pi * 0.7)
#     print((left_distance, right_distance))
#     segment_list.append((left_distance, right_distance))

world_frame_list = []
cur_theta = np.pi / 2
# point_list = point_list[100:200]
for i in range(len(point_list)):
    xr = 0.0318 * point_list[i][2] / (2 * 0.07) + 0.0318 * point_list[i][1] / (2 * 0.066)  
    yr = 0
    thetar = 0.0318 * point_list[i][2] / (2 * 0.05 * 0.7) - 0.0318 * point_list[i][1] / (2 * 0.05 * 0.66)
    robot_frame = np.array([xr, yr, thetar])
    cur_theta += thetar * 0.05
    rotation_mat = np.array([[np.cos(cur_theta), -np.sin(cur_theta), 0], [np.sin(cur_theta), np.cos(cur_theta), 0], [0, 0, 1]])
    world_frame = rotation_mat @ robot_frame.T
    world_frame_list.append(world_frame)

print(world_frame_list)

x, y, theta = 0, 0, 0
trajectory = [(x, y)]

for dx, dy, dtheta in world_frame_list:
    theta += dtheta * 0.05
    x += (dx * np.cos(theta) - dy * np.sin(theta)) * 0.05
    y += (dx * np.sin(theta) + dy * np.cos(theta)) * 0.05

    trajectory.append((x, y))

traj = np.array(trajectory)

# print(traj)
plt.figure(figsize=(8,6))
plt.plot(traj[:, 0], traj[:, 1], marker='o')
# plt.quiver(traj[:-1, 0], traj[:-1, 1],
#            np.diff(traj[:, 0]), np.diff(traj[:, 1]),
#            angles='xy', scale_units='xy', scale=1, color='red', width='0.005')

plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('2D Robot Path')
plt.grid()
plt.legend()
plt.axis('equal')
plt.show()

bag.close()
