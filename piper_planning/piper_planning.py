import numpy as np
import matplotlib.pyplot as plt
import ikpy.chain
import ikpy.utils.plot as plot_utils
from dm_control import mujoco
import time
import transformations as tf
import cv2
from rrt_piper import *
import mujoco as mujoco_n


if __name__ == "__main__":

    # 从URDF文件加载机械臂的运动学连接
    my_chain = ikpy.chain.Chain.from_urdf_file("assets/piper_n.urdf")

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


    num_runs = 1

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

        # 
        joint_limits = [(-3, 3)] * 6
        joint_limits[0] = (-2.687, 2.687)
        joint_limits[1] = (0.0, 3.403) 
        joint_limits[2] = (-3.0541012, 0.0) 
        joint_limits[3] = (-1.8499, 1.8499) 
        joint_limits[4] = (-1.3089, 1.3089) 
        joint_limits[5] = (-1.7452, 1.7452) 

        start_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Start joint angles
        goal_joint = joint_angles[1:]
        # ik -> rrt
        rrt = RRT(start_joint, goal_joint, joint_limits)
        rrt_path = rrt.planning(model)  # Generate the RRT path

        if rrt_path:
            print("Path found!")
            # 设置图形参数
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(0, 1)
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.set_zlabel('Z axis')
            ax.set_title(f'Robot Arm IK Solution - Run {run+1}')
            ax.legend()

            for point in rrt_path:
                # print(point)  # 打印每个路径点
                joint_0 = point[0]
                joint_1 = point[1]
                joint_2 = point[2]
                joint_3 = point[3]
                joint_4 = point[4]
                joint_5 = point[5]

                cur_joints = [0.0, joint_0, joint_1, joint_2, joint_3, joint_4, joint_5]

                my_chain.plot(cur_joints, ax, target=target_position)
            
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

                
                # 暂停一下以便观察
                plt.pause(0.1)


    # 保持图形显示
    plt.show()

        































    
    # 清除之前的绘图
    ax.clear()

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