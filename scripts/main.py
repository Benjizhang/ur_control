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
    # [one option] -0.45285332563724756,0.09539642989562534,0.09082724706215531
    # initPtx = -0.45285332563724756
    # initPty = 0.09539642989562534
    # initPtz = 0.09082724706215531
    # [one option] 
    # initPtx = -0.39906132750470524
    # initPty = 0.09771038592216241
    # initPtz = 0.09082724706215531
    
    # x positive/ x negative/ y positive/ y negative
    xp = 0.23
    xn = 0.45
    yp = 0.22
    yn = 0
    
    # limit of x,y (in world frame)
    xmax = initPtx + yn
    xmin = initPtx - yp
    ymax = initPty + xp
    ymin = initPty - xn
    lim = [xmin, xmax, ymin, ymax]
    # # set zero to the ft 300
    zero_ft_sensor()
    
    # cal. each target
    Lrang = 0.1
    N = 2
    theta_s = 30
    delta_theta = (90 - theta_s)/N
    saftz = initPtz + 0.10
    delta_dep = -0.05
    depthz= initPtz + delta_dep

    # # go home pos
    # ur_control.group.set_named_target('home')
    # ur_control.group.go()
    # rospy.sleep(1)

    # # set joint goal
    # joint_goal = ur_control.group.get_current_joint_values()
    # joint_goal[0] = 0
    # ur_control.group.go(joint_goal, wait=True)
    # ur_control.group.stop()

    # # work aroud the worksapce
    # move_along_boundary(lim)

    # # go the initial position
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

    # rosbag to record the force
    # bag = rosbag.Bag('force_val_dir.bag', 'w')
    listener = listener()
    f_safe = 10

    #region： #experiment of 5 strokes    
    # for i in range(2*N+1):
    #     # current angle (deg)
    #     theta_cur = theta_s + i* delta_theta
    #     # current goal point (in task frame)
    #     x_e_tskf = Lrang*math.cos(np.pi*theta_cur/180)
    #     y_e_tskf = Lrang*math.sin(np.pi*theta_cur/180)
    #     # current goal point (in world frame)
    #     x_e_wldf = initPtx - y_e_tskf
    #     y_e_wldf = initPty + x_e_tskf       

    #     # check the coorrdinate limit
    #     if checkCoorLimit([x_e_wldf, y_e_wldf], lim):
    #         waypoints = []
    #         wpose = ur_control.group.get_current_pose().pose
    #         # lift up
    #         wpose.position.z = saftz
    #         # wpose.position.z += 0.10
    #         waypoints.append(copy.deepcopy(wpose))
    #         # move to the goal
    #         wpose.position.x = x_e_wldf
    #         wpose.position.y = y_e_wldf
    #         waypoints.append(copy.deepcopy(wpose))

    #         # penetration
    #         wpose.position.z = depthz
    #         # wpose.position.z -= 0.1
    #         waypoints.append(copy.deepcopy(wpose))
    #         wpose.position.x += 0.01
    #         wpose.position.y += 0.01
    #         waypoints.append(copy.deepcopy(wpose))
    #         wpose.position.x -= 0.01
    #         wpose.position.y -= 0.01
    #         waypoints.append(copy.deepcopy(wpose))
    #         wpose.position.x -= 0.01
    #         wpose.position.y += 0.01
    #         waypoints.append(copy.deepcopy(wpose))
    #         wpose.position.x += 0.01
    #         wpose.position.y -= 0.01
    #         waypoints.append(copy.deepcopy(wpose))
    #         (plan, fraction) = ur_control.group.compute_cartesian_path(
    #                             waypoints,   # waypoints to follow
    #                             0.01,        # eef_step
    #                             0.0)
    #         ur_control.group.execute(plan, wait=True)
    #         ur_control.group.stop()
    #         rospy.sleep(2)

    #         # go to the object
    #         waypoints = []
    #         wpose.position.x = initPtx
    #         wpose.position.y = initPty
    #         waypoints.append(copy.deepcopy(wpose))
    #         # # start to record the data from Ft300
    #         (plan, fraction) = ur_control.group.compute_cartesian_path(
    #                             waypoints,   # waypoints to follow
    #                             0.01,        # eef_step
    #                             0.0)
    #         listener.clear_finish_flag()
    #         zero_ft_sensor()
    #         # ur_control.group.execute(plan, wait=True)
    #         ur_control.group.execute(plan, wait=False)
    #         rospy.loginfo('clear_finish_flag')
    #         while not listener.read_finish_flag():
    #             rospy.loginfo('Cur X: {}'.format(ur_control.group.get_current_pose().pose.position.x))
    #             rospy.loginfo('Cur Y: {}'.format(ur_control.group.get_current_pose().pose.position.y))
    #             if listener.get_force_val() is not None:
    #                 # rospy.sleep(0.1)
    #                 f_val = listener.get_force_val()
    #                 f_dir = listener.get_force_dir()
    #                 rospy.loginfo('Force Val( N ): {}'.format(f_val))
    #                 rospy.loginfo('Force Dir(deg): {}'.format(f_dir))
    #                 if f_val > f_safe:
    #                     rospy.loginfo('==== Large Force Warning ==== \n')
    #                     ur_control.group.stop()
    #                     break
    #                 with open('/home/zhangzeqing/test{}.csv'.format(i),'a',newline="\n")as f:
    #                     f_csv = csv.writer(f)
    #                     f_csv.writerow([f_val, f_dir])
    #                 rospy.sleep(0.1)

    #         rospy.loginfo('Ref Dir(deg): {}'.format(theta_cur))
    #         rospy.loginfo('{}-th loop finished'.format(i))
            
    #         # ur_control.group.stop()
    #         rospy.sleep(5)

    #     else:
    #         rospy.loginfo('out of the worksapce:\n{}'.format(x_e_wldf,y_e_wldf))
    #         ur_control.group.stop()

    # # lift up
    # waypoints = []
    # wpose = ur_control.group.get_current_pose().pose
    # wpose.position.z = saftz
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = ur_control.group.compute_cartesian_path(
    #                     waypoints,   # waypoints to follow
    #                     0.01,        # eef_step
    #                     0.0)
    # ur_control.group.execute(plan, wait=True)    
    #endregion
    
    # region: straight movement
    # initial (-0.5181971177386158, 0.2430201440263177, -0.0021684198900551316)
    # contact point (-0.47321170688028935, -0.09287122585385203, -0.011764603861335071)
    waypoints = []
    wpose.position.x = -0.5181971177386158-0.1
    wpose.position.y = 0.2430201440263177-0.5
    wpose.position.z = -0.011764603861335071
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)
    ur_control.group.execute(plan, wait=True)
    
    # penetration    
    waypoints = []
    wpose.position.z -= 0.03
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += 0.01
    wpose.position.y += 0.01
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x -= 0.01
    wpose.position.y -= 0.01
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x -= 0.01
    wpose.position.y += 0.01
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x += 0.01
    wpose.position.y -= 0.01
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)
    ur_control.group.execute(plan, wait=True)
    rospy.sleep(2)

    # move right
    waypoints = []
    wpose.position.y += 0.5
    waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x += 0.2
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.y -= 0.5
    # waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)
    listener.clear_finish_flag()
    zero_ft_sensor()
    ur_control.group.execute(plan, wait=False)
    rospy.loginfo('clear_finish_flag')
    while not listener.read_finish_flag():
        if listener.get_force_val() is not None:
            f_val = listener.get_force_val()
            f_dir = listener.get_force_dir()
            rospy.loginfo('Force Val( N ): {}'.format(f_val))
            rospy.loginfo('Force Dir(deg): {}'.format(f_dir))
            if f_val > f_safe:
                rospy.loginfo('==== Large Force Warning ==== \n')
                ur_control.group.stop()
                break
            with open('/home/zhangzeqing/Nutstore Files/Nutstore/line_near_exp4.csv','a',newline="\n")as f:
                f_csv = csv.writer(f)
                f_csv.writerow([f_val, f_dir])
    
    # # lift up
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(
                        waypoints,   # waypoints to follow
                        0.01,        # eef_step
                        0.0)
    ur_control.group.execute(plan, wait=True)

    rospy.loginfo('shut down')




    # xyzquat = ur_control.get_pose_list()
    # T_baselink2tool = helper.xyzquat_to_mat44(xyzquat)
    # xyzrpy_deg = helper.mat44_to_xyzrpy_deg(T_baselink2tool)
    # xyzrpy = copy.deepcopy(xyzrpy_deg)
    # xyzrpy[3:] = np.deg2rad([-180,0,-45])
    # pose_goal = helper.mat44_to_xyzquat(helper.xyzrpy2mat44(xyzrpy))
    # ready_pos = [0.45, 0.4, 0.1, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
    # end_pos = [0.55, 0.3, 0.03, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
    # start_pos = [0.45, 0.4, 0.03, 0.9239015802541878, -0.3826301982273228, 3.707584038430941e-05, 5.8065731946048344e-06]
    # waypoints = []
    # waypoints.append(helper.gen_pose(*ready_pos))
    # (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    # waypoints = []
    # waypoints.append(helper.gen_pose(*start_pos))
    # (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    # waypoints = []
    # waypoints.append(helper.gen_pose(*end_pos))
    # (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    # waypoints = []
    # waypoints.append(helper.gen_pose(*ready_pos))
    # (plan, fraction) = ur_control.go_cartesian_path(waypoints)


    # JS_START =[0.4438713490962982, -1.3743627707110804, 1.543337345123291, 1.33539879322052, 1.5996168851852417, 2.806729793548584]
    # ur_control.go_to_joint_state(*JS_START)
    # # rotation matrix from box {b} to the robot {r}
    # elem = np.sqrt(2)/2
    # rRb = np.array([[elem, elem],
    #                 [-elem, elem]])
    # lx = 0.28
    # ly = 0
    # forward_vec_b = np.array([lx, ly])
    # forward_vec_r = (np.dot(rRb, forward_vec_b)).tolist()

    # waypoints = []
    # # start pt1
    # start1_pose = [0.5285595797087597, 0.33286161521460467, 0.1501572777718378, -0.9219784480530051, 0.38460944184461326, 0.03746876472868755, 0.02504815840443044]
    # #start1_pose[2] -= 0.017
    # waypoints.append(helper.gen_pose(*start1_pose))

    # ur_control.go_to_pose_goal(helper.gen_pose(*start1_pose))

    # (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    # rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
    # if plan is None:
    #     rospy.logerr('plan traj failed')

    # # end pt1
    # end1_pose = copy.deepcopy(start1_pose)
    # end1_pose[0] += forward_vec_r[0]
    # end1_pose[1] += forward_vec_r[1]
    # waypoints.append(helper.gen_pose(*end1_pose))
################################################################


    # # start pt2
    # lx = 0.0
    # ly = 0.06
    # slide_vec_b = np.array([lx, ly])
    # slide_vec_r = (np.dot(rRb, slide_vec_b)).tolist()
    # start2_pose = copy.deepcopy(start1_pose)
    # start2_pose[0] += slide_vec_r[0]
    # start2_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*start2_pose))
    #
    # # end pt2
    # end2_pose = copy.deepcopy(end1_pose)
    # end2_pose[0] += slide_vec_r[0]
    # end2_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*end2_pose))

    # # start pt3
    # start3_pose = copy.deepcopy(start2_pose)
    # start3_pose[0] += slide_vec_r[0]
    # start3_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*start3_pose))
    # # end pt3
    # end3_pose = copy.deepcopy(end2_pose)
    # end3_pose[0] += slide_vec_r[0]
    # end3_pose[1] += slide_vec_r[1]
    # waypoints.append(helper.gen_pose(*end3_pose))

################################################################
    # rospy.loginfo("Start Loop ============================")
    # rate = rospy.Rate(30.0)
    # depth = 0
    # cnt = 0

    # # while not rospy.is_shutdown():
    # for i in range(100):
    #     wpose = []
    #     depth = -0.005 # 5mm deeper
    #     for waypt in waypoints:
    #         waypt.position.z += depth
    #         wpose.append(copy.deepcopy(waypt))

    #     (plan, fraction) = ur_control.go_cartesian_path(wpose)
    #     rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
    #     if plan is None:
    #         rospy.logerr('plan traj failed')
    #         exit(0)
################################################################

    #
    # start1_pose = [0.2700405492584124, 0.37238332698040844, 0.07725913594747941, -0.2606191842360539, 0.6443504900377716, 0.25797526467862947, 0.671072909310314]
    # end1_pose = [0.44333889398164206, 0.18649866354520614, 0.0615933580899586, -0.27145920224433784, 0.6725163143781285, 0.24662990765052872, 0.6428105452342999]
    # start2_pose = [0.27000404407539536, 0.3723622577321929, 0.05721394410574203, -0.27135752101103977, 0.6724648950007767, 0.24655319614995433, 0.6429366860357089]
    # end2_pose = [0.5261471413079262, 0.26371751069459637, 0.05571437862960854, -0.27119477840445955, 0.6724445095241685, 0.24677723247904926, 0.6429407214564483]
    #
    # goal_pose = copy.deepcopy(start1_pose)
    # waypoints = helper.gen_pose(*goal_pose)
    # waypoints.position.z += 0.02  # First move up (z)
    # (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    # if plan is None:
    #     rospy.logerr('plan traj failed')
    # exit(0)



    # # JS_START = [-1.6007002035724085, -1.7271001974688929, -2.2029998938189905, -0.8079999128924769, 1.5951000452041626, -0.03099996248354131]
    # # JS_START = [0.5693637728691101, -1.0146344343768519, 1.6008195877075195, -0.5721419493304651, 1.4757764339447021, 1.5631433725357056]
    # JS_START = [0.7037186026573181, -1.1263330618487757, 1.8121705055236816, -0.614563290272848, 1.4962608814239502, 1.542791485786438]
    #
    # rospy.loginfo("go to init joint")
    # ur_control.go_to_joint_state(*JS_START)
    # rospy.sleep(1)
    #
    # #
    # res = ur_control.set_speed_slider(0.65)  #speed control
    # rospy.loginfo("set_speed_slider: {}".format(res))
    # rospy.sleep(1)
    #
    # rospy.loginfo("Start Loop ============================")
    # rate = rospy.Rate(30.0)
    # while not rospy.is_shutdown():
    #     run = listener.read_and_reset_run()
    #     if True or run:
    #         # pass
    #         # rospy.loginfo("Run --------------------")
    #         poses = listener.read_pose_array()
    #         if poses is None:
    #             rate.sleep()
    #             continue
    #         rospy.loginfo("Rec")
    #         (plan, fraction) = ur_control.go_cartesian_path(helper.gen_pose(*(poses[0])))
    #         if plan is None:
    #             rospy.logerr('plan to init pose failed')
    #
    #         waypoints = []
    #         for pose in poses:
    #             # pose = helper.xyzquat_to_mat44(pose)
    #             # pose = pose.dot(tfs.euler_matrix(*np.deg2rad([180, 0, 0])))
    #             # pose = helper.mat44_to_xyzquat(pose)
    #             waypoints.append(helper.gen_pose(*pose))
    #
    #         goal_pose =[0.35339544678026097, 0.4526103713972721, 0.12411307106246072, -0.6702878730504156, -0.247036939956884, 0.28816145022362866, 0.6376910663819154]
    #         waypoints = helper.gen_pose(*goal_pose)
    #         (plan, fraction) = ur_control.go_cartesian_path(waypoints)
    #         if plan is None:
    #             rospy.logerr('plan traj failed')
    #         exit(0)
    #     rate.sleep()

    # ur_control.group.get_named_target_values('up')
    # ur_control.group.get_remembered_joint_values()
    # ur_control.group.set_joint_value_target('home')

    # ur_control.group.set_pose_reference_frame('base_link')

    # initial_pose = helper.gen_pose(*poses[0])
    # (plan, fraction) = ur_control.go_cartesian_path(initial_pose)


    # rospy.loginfo('shut down')