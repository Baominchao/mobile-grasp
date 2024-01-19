#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import intera_interface
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from std_msgs.msg import Header
from rs_yolo.msg import Info
from sensor_msgs.msg import JointState
import numpy as np
import tf

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import Bool


camera_temp = Info()
camera_temp_list = []
nav_isok = False
n = 0

def nav_callback(msg):
    global nav_isok, n
    n = n + 1
    if msg.status.status == 3 and n == 1 :
        nav_isok = True

# trans_camera_to_base = np.array([0.07063020834009065, -0.035085132673118725, -0.16908533701194134]) # xyz
# rot_camera_to_base = np.array([0.003688922548136208, 0.006070583531063267, 0.7121696086608277, 0.70197150110664]) #xyzw
# 标定结果中的平移和旋转信息
trans_camera_to_base = np.array([0.06917871000620401, -0.03498715416826891, -0.12019276944683992]) # xyz
rot_camera_to_base = np.array([-0.0007041243552384791, 0.005115955142647288, 0.7144406247543598, 0.6996770147092657]) #xyzw

# end_effector -->> base
def transform_end_effector_to_base(end_effector_point):
    try:
        # 等待TF数据的更新
        tf_listener.waitForTransform("base", "right_gripper_tip", rospy.Time(0), rospy.Duration(0))
        
        # 进行坐标转换
        base_point = tf_listener.transformPoint("base", end_effector_point)
        

        if base_point == None:
            return False

        return base_point.point.x, base_point.point.y, base_point.point.z
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF transform failed: %s", str(e))
        return False
    

# 相机坐标点转换到机器人base坐标(eye to hand)  //  camera -->> end_effector(eye in hand)
def transform(x, y, z):
    rot_camera_to_base_matrix = tf.transformations.quaternion_matrix(rot_camera_to_base)[:3, :3]
    object_position_camera = np.array([x, y, z])
    object_position_end = trans_camera_to_base + np.dot(rot_camera_to_base_matrix, object_position_camera)
    # 创建末端执行器的坐标
    end_effector_point = PointStamped()
    end_effector_point.header = Header(stamp=rospy.Time.now(), frame_id='right_gripper_tip') #  + rospy.Duration(5.5)
    end_effector_point.point = Point(object_position_end[0], object_position_end[1], object_position_end[2])

    # 进行末端执行器到基座的坐标转换
    if transform_end_effector_to_base(end_effector_point) == False :
        return False
    base_x, base_y, base_z = transform_end_effector_to_base(end_effector_point)
    if base_x is not None and base_y is not None and base_z is not None:
        #rospy.loginfo("End Effector in base_link coordinates: x=%f, y=%f, z=%f", base_x, base_y, base_z)
        pass
    return [base_x, base_y, base_z]



def ik_service_client(target_pose, arm, limb = "right", use_advanced_options = False):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
   
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(target_pose)
    # Request inverse kinematics from base to "right_gripper_tip" link
    ikreq.tip_names.append('right_gripper_tip')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        rospy.loginfo("Running Advanced IK Service Client example.")
        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [0.1, -0.3, 0.5]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.4)
    else:
        rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        # seed_str = {
        #             ikreq.SEED_USER: 'User Provided Seed',
        #             ikreq.SEED_CURRENT: 'Current Joint Angles',
        #             ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
        #            }.get(resp.result_type[0], 'None')
        # rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
        #       (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(list(zip(resp.joints[0].name, resp.joints[0].position)))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)
        
        # Move to the IK solution
        arm.move_to_joint_positions(limb_joints)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return False

    return True

def detect_callback(msg):
    global camera_temp, camera_temp_list
    camera_temp = msg
    # 将 camera_temp.data 转换为列表
    camera_temp_list = list(camera_temp.data)
    #print(camera_temp_list)
    # 判断是否是4的余数为0 (x,y,z,class_id)
    if len(camera_temp_list) % 4 != 0:
        #print('视觉话题输出错误,不是4的倍数')
        return None

    # 提取xyz并进行相机坐标->>base坐标转换

    transformed_coordinates = [(camera_temp_list[i], camera_temp_list[i + 1], camera_temp_list[i + 2]) for i in range(0, len(camera_temp_list), 4)]
    transformed_coordinates = [transform(x, y, z) for x, y, z in transformed_coordinates]

    # 将转换后的坐标重新放回 camera_temp_list
    for i in range(len(camera_temp_list) // 4):
        camera_temp_list[i * 4], camera_temp_list[i * 4 + 1], camera_temp_list[i * 4 + 2] = transformed_coordinates[i]

class ants():
    def __init__(self):
        # 创建Sawyer机器人的接口
        self.limb = intera_interface.Limb('right')
        self.limb.set_joint_position_speed(0.2)
        # 夹爪开关 
        self.gripper = intera_interface.Gripper('right_gripper')
        rospy.sleep(1)
        self.gripper.open()
        rospy.sleep(3)
        # 订阅视觉检测结果
        self.sub_vision =  rospy.Subscriber("/detect_result_out", Info, detect_callback, queue_size=10)
 
        camera_joint_angles = {
        'right_j6': -1.60992675781,
        'right_j5': 1.11107714844,
        'right_j4': -0.0019423828125,
        'right_j3': 2.07815332031,
        'right_j2': 0.164627929688,
        'right_j1': -1.63870800781,
        'right_j0': 1.35003808594
        }     

        #self.limb.move_to_joint_positions(home_joint_angles)
        
        self.limb.move_to_joint_positions(camera_joint_angles)
        rospy.sleep(2)

        # 坐标列表
        self.camera_list = []

        # camera_joint_angles = {
        # 'right_j6': -1.52394921875,
        # 'right_j5': 0.924275390625,
        # 'right_j4': 0.115829101562,
        # 'right_j3': 1.68639941406,
        # 'right_j2': -0.119344726563,
        # 'right_j1': -0.992592773437,
        # 'right_j0': 0.0658662109375
        # }

        
    
    def correct_position(self):
        camera_joint_angles = {
        'right_j6': -1.393046875,
        'right_j5': 1.90675292969,
        'right_j4': 0.104055664062,
        'right_j3': 0.95241015625,
        'right_j2': -0.294555664062,
        'right_j1': -1.26809472656,
        'right_j0': 0.35237109375
        }
        self.limb.move_to_joint_positions(camera_joint_angles)
        rospy.sleep(10)
        # # while not self.car_cmd(camera_temp_list):
        # #         pass
        self.camera_list = camera_temp_list  # 条件成立后，才会把正确的坐标给到camera变量
        #print(self.camera_list)


    def car_cmd(self, camera_temp_list):
            vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
            rospy.sleep(1)
            cmd = Twist()

            camera_groups = [camera_temp_list[i:i+4] for i in range(0, len(camera_temp_list), 4)]

            # 找到 x 最小的整体部分
            camera_temp = min(camera_groups, key=lambda group: group[0])

            if camera_temp[0] > 0.6:
                cmd.linear.x = 0.03
                vel_pub.publish(cmd)
                return False
            if camera_temp[0] < 0.45:
                cmd.linear.x = -0.05
                vel_pub.publish(cmd)
                return False
            if camera_temp[1] < 0.05:
                cmd.linear.y = -0.02
                vel_pub.publish(cmd)
                return False
            if camera_temp[1] > -0.05:
                cmd.linear.y = 0.02
                vel_pub.publish(cmd)
                return False
            return True
    

    # 示例条件判断函数
    def should_process(self, element4):
        # 示例索引列表
        index_list = ['apple', 'banana', 'dog', 'mango']
        # 将浮点数转换为整数
        int_element4 = int(element4)
        # 判断索引是否在列表范围内
        return 0 <= int_element4 < len(index_list)

    def process_and_remove(self):
        i = 0
        while i < len(self.camera_list):
            # 检查是否有足够的元素用于提取
            if i + 3 < len(self.camera_list):
                # 提取四个元素
                element1 = self.camera_list[i]
                element2 = self.camera_list[i + 1]
                element3 = self.camera_list[i + 2]
                element4 = self.camera_list[i + 3]

                # 在这里执行你的处理逻辑，使用 element1 到 element4
                if self.should_process(element4):  # 替换为实际的条件判断函数
                    #print("Processing elements: {}, {}, {}".format(element1, element2, element3))
                    if element4 == 2:
                        continue
                    self.move_sawyer_arm(element1, element2, element3, element4)
                    # 处理完后从列表中删除这四个元素
                    del self.camera_list[i:i + 4]
                else:
                    i += 4  # 如果不处理，移动到下一个四元组
            else:
                print("Not enough elements to process")
                break  # 结束循环，因为剩余元素不足四个

   


    def move_sawyer_arm(self, X,Y,Z, class_id): 
        # # # 标定误差补偿 kalibr 1
        # X = X
        # Y = Y + 0.01
        # Z = Z - 0.015

        # # # # 标定误差补偿 kalibr 3
        # X = X - 0.015
        # Y = Y + 0.01
        # Z = Z + 0.03

        # # 标定误差补偿 ROS 5
        # X = X - 0.01
        # Y = Y + 0.002
        # Z = Z

        X = X - 0.01
        Y = Y
        Z = Z + 0.025

        class_id = int(class_id)
        # 示例索引列表
        index_list = ['apple', 'banana', 'dog', 'mango']
        # 创建映射字典
        depth_mapping = {'apple': 0.03, 'banana': 0.01, 'dog': 0, 'mango': 0.03}
        # 初始化 down_depth,深度信息为检测物体表面中心点，为夹到物体需在z轴向下补偿
        down_depth = None
        # 根据 class_id 判断类别，并赋值给 down_depth
        element_key = index_list[class_id] if 0 <= class_id < len(index_list) else None
        down_depth = depth_mapping.get(element_key)
        
        # 1. 移动到物体上方5厘米
        above_object_pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(
                position=Point(x=X, y=Y, z=Z + 0.05),  # 上方5厘米
                orientation=Quaternion(x=0.9992035860082825, y=0.024334317531235664, z=-0.030862089098445653, w=-0.006896822091206235),
            ),
        )
        ik_service_client(above_object_pose, self.limb)
        rospy.sleep(1)

        #2. 向下抓取
        object_pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(
                position=Point(x=X, y=Y, z=Z - 0.03),  # 物体位置
                orientation=Quaternion(x=0.9992035860082825, y=0.024334317531235664, z=-0.030862089098445653, w=-0.006896822091206235),
            ),
        )
        ik_service_client(object_pose, self.limb)
        self.gripper.close()
        rospy.sleep(1)

        # 3. 抬起来
        above_object_pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(
                position=Point(x=X, y=Y, z=Z + 0.15), 
                orientation=Quaternion(x=0.9992035860082825, y=0.024334317531235664, z=-0.030862089098445653, w=-0.006896822091206235),
            ),
        )
        ik_service_client(above_object_pose, self.limb)
        rospy.sleep(1)

        camera_joint_angles = {
        'right_j6': -1.70772460938,
        'right_j5': 1.868859375,
        'right_j4': -0.02989453125,
        'right_j3': 1.3821953125,
        'right_j2': -0.185561523437,
        'right_j1': -1.66476269531,
        'right_j0': 1.44226367187
        }

        self.limb.move_to_joint_positions(camera_joint_angles)
        rospy.sleep(1)

        camera_joint_angles = {
        'right_j6': -1.6961953125,
        'right_j5': 0.981670898437,
        'right_j4': -0.107346679688,
        'right_j3': 2.04459570313,
        'right_j2': -0.146967773438,
        'right_j1': -1.47723144531,
        'right_j0': 1.4324296875
        }

        self.limb.move_to_joint_positions(camera_joint_angles)
        self.gripper.open()

        # # 4. 移动到固定位置（假设目标位置是X=1, Y=1, Z=1）
        # # 先移动到固定位置上方
        # fixed_position_pose = PoseStamped(
        #     header=Header(stamp=rospy.Time.now(), frame_id='base'),
        #     pose=Pose(
        #         position=Point(x=0, y=0.4, z=0.3 +0.15),
        #         orientation=Quaternion(x=0.9992035860082825, y=0.024334317531235664, z=-0.030862089098445653, w=-0.006896822091206235),
        #     ),
        # )
        # ik_service_client(fixed_position_pose, self.limb)
        # rospy.sleep(2)
        # # 再移动到固定位置
        # fixed_position_pose = PoseStamped(
        #     header=Header(stamp=rospy.Time.now(), frame_id='base'),
        #     pose=Pose(
        #         position=Point(x=0, y=0.4, z=0.3),
        #         orientation=Quaternion(x=0.9992035860082825, y=0.024334317531235664, z=-0.030862089098445653, w=-0.006896822091206235),
        #     ),
        # )
        # ik_service_client(fixed_position_pose, self.limb)
        # self.gripper.open()
        # rospy.sleep(2)

        # # 抬起来
        # fixed_position_pose = PoseStamped(
        #     header=Header(stamp=rospy.Time.now(), frame_id='base'),
        #     pose=Pose(
        #         position=Point(x=0, y=0.4, z=0.3 +0.15),
        #         orientation=Quaternion(x=0.9992035860082825, y=0.024334317531235664, z=-0.030862089098445653, w=-0.006896822091206235),
        #     ),
        # )
        # ik_service_client(fixed_position_pose, self.limb)
        # rospy.sleep(2)

if __name__ == '__main__':

     # 初始化ROS节点
    rospy.init_node('move_sawyer_arm', anonymous=True)

    # 订阅导航结果
    result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, nav_callback)

    # 发布抓取完毕
    grasp_pub = rospy.Publisher("/grasp_over", Bool, queue_size=10)

    # 创建 TF TransformListener
    tf_listener = tf.TransformListener()

    robot = ants()
    #robot.correct_position()
    #robot.process_and_remove()
    grasp_isok = Bool()
    grasp_isok.data = False
    while not nav_isok:
        rospy.sleep(1)  # 短暂休眠以避免过度占用CPU
    if nav_isok:
        robot.correct_position()
        robot.process_and_remove()
        grasp_isok.data = True
        camera_joint_angles = {
        'right_j6': -1.08628222656,
        'right_j5': -2.12968457031,
        'right_j4': -0.26789453125,
        'right_j3': 2.3815390625,
        'right_j2': 8.49609375e-05,
        'right_j1': -1.2496484375,
        'right_j0': 1.23706347656
        }

        robot.limb.move_to_joint_positions(camera_joint_angles)
        rospy.sleep(8)
        grasp_pub.publish(grasp_isok)
        

    while not (rospy.is_shutdown()):
        pass
