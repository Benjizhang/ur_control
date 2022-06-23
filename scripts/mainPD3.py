# for 10 exp., where the probe slides along 5 angles in each exp.
# Penetration Depth: 3cm
#
# Z. Zhang
# 06/22

from cmath import cos
import copy
import threading

# import apriltag_ros.msg
import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseArray, TransformStamped
from std_msgs.msg import String

import helper
from tf import transformations as tfs
from scene_helper import setup_scene
from ur_move import MoveGroupPythonInteface
from robotiq_ft_sensor.msg import ft_sensor
from control_msgs.msg import FollowJointTrajectoryActionResult as rlst
import math
import moveit_commander
import sys
import rosbag
import csv

class listener():
    def __init__(self):
        # self.tfb = tf.TransformBroadcaster()
        # self.stfb = tf2_ros.StaticTransformBroadcaster()
        # listener = tf.TransformListener()
        #
        # # Get static tf
        # listener.waitForTransform('base', 'base_link', rospy.Time(0), rospy.Duration(1))
        # tr_base2baselink = listener.lookupTransform('base', 'base_link', rospy.Time(0))
        # tf_base2baselink = listener.fromTranslationRotation(*tr_base2baselink)
        # self.tf_base2baselink = tf_base2baselink
        #
        # listener.waitForTransform('base_link', 'camera', rospy.Time(0), rospy.Duration(1))
        # tr_baselink2camera = listener.lookupTransform('base_link', 'camera', rospy.Time(0))
        # tf_baselink2camera = listener.fromTranslationRotation(*tr_baselink2camera)
        # self.tf_baselink2cam = tf_baselink2camera
        #
        # self.tf_ee2tool = helper.xyzrpy2mat44([0.09, 0, 0, np.deg2rad([-90, 0, -90])])
        # self.tf_tool2ee = np.linalg.inv(self.tf_ee2tool)
        #
        # xyzquat = helper.mat44_to_xyzquat(self.tf_ee2tool)
        # static_transformStamped = TransformStamped()
        # static_transformStamped.header.stamp = rospy.Time.now()
        # static_transformStamped.header.frame_id = 'ee_link'
        # static_transformStamped.child_frame_id = 'tool_vacuum'
        # static_transformStamped.transform.translation.x=xyzquat[0]
        # static_transformStamped.transform.translation.y=xyzquat[1]
        # static_transformStamped.transform.translation.z=xyzquat[2]
        # static_transformStamped.transform.rotation.x=xyzquat[3]
        # static_transformStamped.transform.rotation.y=xyzquat[4]
        # static_transformStamped.transform.rotation.z=xyzquat[5]
        # static_transformStamped.transform.rotation.w=xyzquat[6]
        # self.stfb.sendTransform(static_transformStamped)
        #
        # self.tf_listener = listener

        # self.count = 0

        # self.plot_pub = rospy.Publisher("plot", geometry_msgs.msg.PointStamped, queue_size=2)

        # ns = '/obj_detect/'
        self.obj_pose_sub = rospy.Subscriber("robotiq_ft_sensor", ft_sensor, self.detect_callbak, queue_size=1)
        self.result_status = rospy.Subscriber("/scaled_pos_joint_traj_controller/follow_joint_trajectory/result",rlst,self.callback, queue_size=1)
        self.lock_read = threading.Lock()
        # self.run_flag = False
        # self.gripper_pos = 80   # !!!!!!!!!!! Default grip pos
        # self.chat_sub = rospy.Subscriber("/debug_chat", String, self.chat_callbak, queue_size=1)
        # self.posearray_sub = rospy.Subscriber("/objpose", PoseArray, self.pose_callbak, queue_size=2)
        # self.pose_array=None
        self.sensor_data = None
        self.force_val = None
        self.force_dir = None
        self.plan_finished =False

    def detect_callbak(self,msg):
        # msg = apriltag_ros.msg.AprilTagDetectionArray()
        # for det in msg.detections:
        #     det.pose
        # a=msg.detections[0].pose.pose.pose
        with self.lock_read:
            # rospy.loginfo(f"I heard: {msg}")
            # self.sensor_data = msg.xxx

            # calculate the force values
            self.force_val = math.sqrt((msg.Fx)**2+(msg.Fy)**2)

            # calculate the force dir. (deg)
            dir_rad = math.atan2(msg.Fy, msg.Fx)
            self.force_dir = math.degrees(dir_rad)     
    
    def callback(self,msg):
        rospy.loginfo('finish callback')
        with self.lock_read:
            self.plan_finished =True
    
    def clear_finish_flag(self):
        with self.lock_read:
            self.plan_finished = False
    
    def read_finish_flag(self):
        with self.lock_read:
            return self.plan_finished
    

    #     # rospy.loginfo(msg)
    
    def read_sensor(self):
        with self.lock_read:
            return self.sensor_data
    
    def get_force_val(self):
        with self.lock_read:
            return self.force_val

    def get_force_dir(self):
        with self.lock_read:
            return self.force_dir

import robotiq_ft_sensor.srv
def zero_ft_sensor():
    srv_name = '/robotiq_ft_sensor_acc'
    rospy.wait_for_service(srv_name)
    try:
        srv_fun = rospy.ServiceProxy(srv_name, robotiq_ft_sensor.srv.sensor_accessor)
        resp1 = srv_fun(robotiq_ft_sensor.srv.sensor_accessorRequest.COMMAND_SET_ZERO, '')
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# check whether in the limit 
def checkCoorLimit(pos, lim):
    curx = pos[0]
    cury = pos[1]
    if curx >= lim[0] and curx <= lim[1] and \
        cury >= lim[2] and cury <= lim[3]:
        return True
    else: return False

def getXYwld_from_tsk(tansl, xy_tsk):
    # fixed orientation of two frames
    Rot_tsk_to_wld = np.mat([[0, -1],[1, 0]])
    tanslation_column = np.reshape(tansl,(2,1))
    xy_tsk_column = np.reshape(xy_tsk,(2,1))
    xy_wld = tansl+Rot_tsk_to_wld*xy_tsk_column
    return xy_wld 

def move_along_boundary(lim):
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    # record start pt
    x_start = wpose.position.x
    y_start = wpose.position.y
    
    # get vertical
    quater_init = tfs.quaternion_from_euler(0, np.pi, np.pi/2,'szyz')
    wpose.orientation.x = quater_init[0]
    wpose.orientation.y = quater_init[1]
    wpose.orientation.z = quater_init[2]
    wpose.orientation.w = quater_init[3]
    waypoints.append(copy.deepcopy(wpose))

    xmin = lim[0]
    xmax = lim[1]
    ymin = lim[2]
    ymax = lim[3]
    # xmin, ymin
    wpose.position.x = xmin
    wpose.position.y = ymin
    waypoints.append(copy.deepcopy(wpose))
    # xmin, ymax
    wpose.position.x = xmin
    wpose.position.y = ymax
    waypoints.append(copy.deepcopy(wpose))
    # xmax, ymax
    wpose.position.x = xmax
    wpose.position.y = ymax
    waypoints.append(copy.deepcopy(wpose))
    # xmax, ymin
    wpose.position.x = xmax
    wpose.position.y = ymin
    waypoints.append(copy.deepcopy(wpose))
    # go back
    wpose.position.x = x_start
    wpose.position.y = y_start
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = ur_control.group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)
    ur_control.group.execute(plan, wait=True)
    ur_control.group.stop()

def emergency_stop(saftz):
    # rospy.sleep(1)
    # ur_control.group.stop()
    rospy.loginfo('==== Emergency Stop ==== \n')     
    # quick lift up ！！！！
    rospy.loginfo('==== Lift Up ({:.2f}) ==== \n'.format(saftz))
    res = ur_control.set_speed_slider(0.3)
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z = saftz
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(
                        waypoints,   # waypoints to follow
                        0.01,        # eef_step
                        0.0)
    ur_control.group.execute(plan, wait=True)   
    return True


if __name__ == '__main__':
    rospy.init_node("test_move")
    
    # rospy.loginfo('zeroed')
    # rospy.spin()
    moveit_commander.roscpp_initialize(sys.argv)
    enableFt300 = 0

    ############# ft 300 #############
    if enableFt300 == 1:
        listener = listener()
        while listener.get_force_val() is None:
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            force = listener.read_sensor()
            f_val = listener.get_force_val()
            f_dir = listener.get_force_dir()

            if f_val > 8:
                print("force is large")
                # ur_control.stop()

            rospy.sleep(0.03)


    ################################################################
    ############# ur control #############
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = "ur5"
    # group = moveit_commander.MoveGroupCommander(group_name)

    # ur_control = MoveGroupPythonInteface(sim=True)  #simu
    ur_control = MoveGroupPythonInteface(sim=False)  #real
    rospy.loginfo('init ok.')

    ur_control.group.get_planning_frame()
    ur_control.group.get_end_effector_link()

    ur_control.remove_allobjects()
    # setup_scene(ur_control)

    # Config UR
    res = ur_control.play_program()
    rospy.loginfo("play_program: {}".format(res))

    res = ur_control.set_speed_slider(0.3)
    rospy.loginfo("set_speed_slider: {}".format(res))
    rospy.sleep(1)

    # obtain the current pose list
    rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
    rospy.loginfo('current joint: \n{}'.format(ur_control.get_joint_pos_list()))

    # # set the initial pos (i.e., origin of task frame)
    # wpose = ur_control.get_pose_list()
    # startx = wpose[0]
    # starty = wpose[1]
    # group_names = ur_control.robot.get_group_names()
    # print("============ Robot Groups:", ur_control.robot.get_group_names())
    # print("============ Printing robot state",robot.get_current_state())
    initPtx = ur_control.group.get_current_pose().pose.position.x
    initPty = ur_control.group.get_current_pose().pose.position.y
    initPtz = ur_control.group.get_current_pose().pose.position.z # surface plane
    # [one option] 
    # initPtx = 
    # initPty = 
    # initPtz = 
    # [one option] 
    initPtx = -0.4865369575882814
    initPty = -0.08797877559242727
    initPtz = 0.07731254732208744 # -0.011764603861335071
    # x positive/ x negative/ y positive/ y negative
    xp = 0.23
    xn = 0.45
    yp = 0.22
    yn = 0
    
    # limit of x,y (in world frame)
    # xmax = initPtx + yn
    # xmin = initPtx - yp
    # ymax = initPty + xp
    # ymin = initPty - xn
    xmax = initPtx + 1
    xmin = initPtx - 1
    ymax = initPty + 1
    ymin = initPty - 1
    lim = [xmin, xmax, ymin, ymax]
    # # set zero to the ft 300
    zero_ft_sensor()
    
    # cal. each target
    Lrang = 0.1
    N = 2
    theta_s = 30
    delta_theta = (90 - theta_s)/N
    saftz = initPtz + 0.10
    # PENETRATION DEPTH
    PENE_DEPTH = -0.03 #(default: -0.03)
    depthz = initPtz + PENE_DEPTH
    # SAFE FORCE
    SAFE_FORCE = 15.0
    flargeFlag = 0
    listener = listener()    

    for j in range(0,10):
        #go the initial position
        waypoints = []
        wpose = ur_control.group.get_current_pose().pose
        wpose.position.x = initPtx
        wpose.position.y = initPty
        wpose.position.z = initPtz    
        quater_init = tfs.quaternion_from_euler(0, np.pi, np.pi/2,'szyz')
        wpose.orientation.x = quater_init[0]
        wpose.orientation.y = quater_init[1]
        wpose.orientation.z = quater_init[2]
        wpose.orientation.w = quater_init[3]
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = ur_control.group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)
        ur_control.group.execute(plan, wait=True)
    
    
        #region： #experiment of 5 strokes    
        for i in range(2*N+1):
            # current angle (deg)
            theta_cur = theta_s + i * delta_theta
            # current goal point (in task frame)
            x_e_tskf = Lrang*math.cos(np.pi*theta_cur/180)
            y_e_tskf = Lrang*math.sin(np.pi*theta_cur/180)
            # current goal point (in world frame)
            x_e_wldf = initPtx - x_e_tskf
            y_e_wldf = initPty - y_e_tskf       

            # check the coorrdinate limit
            if checkCoorLimit([x_e_wldf, y_e_wldf], lim):
                waypoints = []
                wpose = ur_control.group.get_current_pose().pose
                # lift up
                wpose.position.z = saftz
                # wpose.position.z += 0.10
                waypoints.append(copy.deepcopy(wpose))
                # move to the goal
                wpose.position.x = x_e_wldf
                wpose.position.y = y_e_wldf
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = ur_control.group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)
                ur_control.group.execute(plan, wait=True)

                # penetration
                waypoints = []
                wpose.position.z = depthz
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = ur_control.group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)
                ur_control.group.execute(plan, wait=True)
                ur_control.group.stop()
                rospy.sleep(2)

                # go to the object
                waypoints = []
                wpose.position.x = initPtx
                wpose.position.y = initPty
                waypoints.append(copy.deepcopy(wpose))
                # # start to record the data from Ft300
                (plan, fraction) = ur_control.group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)
                listener.clear_finish_flag()
                zero_ft_sensor()
                # ur_control.group.execute(plan, wait=True)
                ur_control.group.execute(plan, wait=False)
                rospy.loginfo('clear_finish_flag')
                while not listener.read_finish_flag():
                    # rospy.loginfo('Cur X: {}'.format(ur_control.group.get_current_pose().pose.position.x))
                    # rospy.loginfo('Cur Y: {}'.format(ur_control.group.get_current_pose().pose.position.y))
                    # rospy.loginfo('Cur Z: {}'.format(ur_control.group.get_current_pose().pose.position.z))
                    if listener.get_force_val() is not None:
                        # rospy.sleep(0.1)
                        # cal. the distance to the obj.
                        ## cur pos.
                        curx = ur_control.group.get_current_pose().pose.position.x
                        cury = ur_control.group.get_current_pose().pose.position.y
                        dist = np.sqrt((initPtx - curx)**2+(initPty - cury)**2)
                        # measure the force val/dir
                        f_val = listener.get_force_val()
                        f_dir = listener.get_force_dir()
                        rospy.loginfo('Force Val( N ): {}'.format(f_val))
                        # rospy.loginfo('Force Dir(deg): {}'.format(f_dir))
                        if f_val > SAFE_FORCE:
                            rospy.loginfo('==== Large Force Warning ==== \n')
                            # emergency stop!!!
                            flargeFlag = emergency_stop(saftz)   
                            break
                        with open('/home/zhangzeqing/Nutstore Files/Nutstore/zeroNormalVector7D3/{}/znv_exp{}.csv'.format(j+1,i),'a',newline="\n")as f:
                            f_csv = csv.writer(f)
                            f_csv.writerow([f_val, f_dir, dist])
                        f.close()

                rospy.loginfo('Ref Dir(deg): {}'.format(theta_cur))
                rospy.loginfo('{}-th loop finished'.format(i))
                
                # ur_control.group.stop()
                # rospy.sleep(3)

            else:
                rospy.loginfo('out of the worksapce:\n{}'.format(x_e_wldf,y_e_wldf))
                ur_control.group.stop()
            if flargeFlag == 1:
                break
        if flargeFlag == 1:
            break
    
    # lift up
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z = saftz
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(
                        waypoints,   # waypoints to follow
                        0.01,        # eef_step
                        0.0)
    ur_control.group.execute(plan, wait=True)    
    # endregion
    
    # region: straight movement
    # # initial (-0.5181971177386158, 0.2430201440263177, -0.0021684198900551316)
    # # contact point (-0.47321170688028935, -0.09287122585385203, -0.011764603861335071)
    # waypoints = []
    # wpose.position.x = -0.47321170688028935
    # wpose.position.y = -0.09287122585385203
    # wpose.position.z = -0.011764603861335071
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = ur_control.group.compute_cartesian_path(
    #                             waypoints,   # waypoints to follow
    #                             0.01,        # eef_step
    #                             0.0)
    # ur_control.group.execute(plan, wait=True)
    
    # # penetration    
    # waypoints = []
    # wpose.position.z -= 0.03
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x += 0.01
    # wpose.position.y += 0.01
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x -= 0.01
    # wpose.position.y -= 0.01
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x -= 0.01
    # wpose.position.y += 0.01
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x += 0.01
    # wpose.position.y -= 0.01
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = ur_control.group.compute_cartesian_path(
    #                             waypoints,   # waypoints to follow
    #                             0.01,        # eef_step
    #                             0.0)
    # ur_control.group.execute(plan, wait=True)
    # rospy.sleep(2)

    # # move right
    # waypoints = []
    # wpose.position.y += 0.5
    # waypoints.append(copy.deepcopy(wpose))
    # # wpose.position.x += 0.2
    # # waypoints.append(copy.deepcopy(wpose))
    # # wpose.position.y -= 0.5
    # # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = ur_control.group.compute_cartesian_path(
    #                             waypoints,   # waypoints to follow
    #                             0.01,        # eef_step
    #                             0.0)
    # listener.clear_finish_flag()
    # zero_ft_sensor()
    # ur_control.group.execute(plan, wait=False)
    # rospy.loginfo('clear_finish_flag')
    # while not listener.read_finish_flag():
    #     if listener.get_force_val() is not None:
    #         f_val = listener.get_force_val()
    #         f_dir = listener.get_force_dir()
    #         rospy.loginfo('Force Val( N ): {}'.format(f_val))
    #         rospy.loginfo('Force Dir(deg): {}'.format(f_dir))
    #         if f_val > f_safe:
    #             rospy.loginfo('==== Large Force Warning ==== \n')
    #             ur_control.group.stop()
    #             break
    #         with open('/home/zhangzeqing/Nutstore Files/Nutstore/line_near_exp4.csv','a',newline="\n")as f:
    #             f_csv = csv.writer(f)
    #             f_csv.writerow([f_val, f_dir])
    
    # # # lift up
    # waypoints = []
    # wpose = ur_control.group.get_current_pose().pose
    # wpose.position.z += 0.1
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = ur_control.group.compute_cartesian_path(
    #                     waypoints,   # waypoints to follow
    #                     0.01,        # eef_step
    #                     0.0)
    # ur_control.group.execute(plan, wait=True)
    # endregion
    rospy.loginfo('shut down')
