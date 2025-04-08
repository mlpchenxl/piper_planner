import numpy as np
import matplotlib.pyplot as plt
import ikpy.chain
import ikpy.utils.plot as plot_utils
from dm_control import mujoco
import time
import transformations as tf
import cv2

# 从URDF文件加载机械臂的运动学连接
my_chain = ikpy.chain.Chain.from_urdf_file("assets/piper_n.urdf")
# my_chain = ikpy.chain.Chain.from_urdf_file("assets/a1_right.urdf")


# # 加载MuJoCo模型
# model = mujoco.Physics.from_xml_path('assets/a1_ik.xml')

# Number of runs
num_runs = 1

# 创建3D图形
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# 设置图形参数
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(0, 1)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
ax.set_title('Robot Arm IK Solution Visualization')

# 主仿真循环
for run in range(num_runs):
    # 生成随机目标位置
    target_position = np.array([0.2, -0.2, 0.35])
    # 生成随机欧拉角方向
    target_orientation_euler = np.array([ 0.98445508, -0.01820646,  0.17469034])
    # 将欧拉角转换为3x3旋转矩阵
    target_orientation = tf.euler_matrix(*target_orientation_euler)[:3, :3]
    print(f"Run {run+1}: {target_position=}, {target_orientation=}")

    # 计算逆运动学解
    joint_angles = my_chain.inverse_kinematics(target_position, target_orientation, "all")
    print("The angles of each joint are:", joint_angles)

    # 验证计算结果
    real_frame = my_chain.forward_kinematics(joint_angles)
    print(f"real_frame type is : {type(real_frame)}")
    print("The real frame is:", real_frame)
    
    # 清除之前的绘图
    ax.clear()
    
    # 绘制机械臂
    # joint_angles = np.array([0, 0, 1.50603839, -0.26118674, 0, -1.13815009, 0])
    # joint_angles = np.array([0, 0, 0, 0, 0, 0, 0])

    my_chain.plot(joint_angles, ax, target=target_position)
    
    # 绘制目标坐标系
    # 创建目标坐标系箭头
    arrow_length = 0.1
    ax.quiver(
        target_position[0], target_position[1], target_position[2],
        target_orientation[0, 0], target_orientation[1, 0], target_orientation[2, 0],
        color='r', length=arrow_length, normalize=True, label='X Target'
    )
    ax.quiver(
        target_position[0], target_position[1], target_position[2],
        target_orientation[0, 1], target_orientation[1, 1], target_orientation[2, 1],
        color='g', length=arrow_length, normalize=True, label='Y Target'
    )
    ax.quiver(
        target_position[0], target_position[1], target_position[2],
        target_orientation[0, 2], target_orientation[1, 2], target_orientation[2, 2],
        color='b', length=arrow_length, normalize=True, label='Z Target'
    )
    
    # 设置图形参数
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(0, 1)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title(f'Robot Arm IK Solution - Run {run+1}')
    ax.legend()
    
    # 暂停一下以便观察
    plt.pause(1)

# 保持图形显示
plt.show()