# exp: ur5 follow spiral trajectory
#
# Penetration Depth: D5cm # <<<<<<
# speed/velocity: 0.1 （slow motion）
# 
# Z. Zhang
# 07/22

from cmath import cos
import copy
import threading

# import apriltag_ros.msg
import numpy as np
from sqlalchemy import true
from sympy import re
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
# sys.path.append("/home/zhangzeqing/ur5_ws/src/ur_control")
import csv
import pylab
from jamming_detector import jamming_detector1 as jd1
from handle_drag_force import smooth_fd_kf, get_mean
import robotiq_ft_sensor.srv
from functions.spiralTraj import ur_spiralTraj

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
        # rospy.loginfo('finish callback')
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
    rospy.loginfo('==== Emergency Stop ==== \n')

    # quick lift up ！！！！
    rospy.loginfo('==== Lift Up ({:.2f}) ==== \n'.format(saftz))
    res = ur_control.set_speed_slider(0.01)
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
    ur_control.group.execute(plan, wait=True)

    res = ur_control.set_speed_slider(0.4)
    waypoints = []
    wpose.position.z = saftz
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
    ur_control.group.execute(plan, wait=True)   
    return True

def emergency_stop2(stop_pos, saftz):
    rospy.loginfo('==== Emergency Stop ==== \n')

    # quick lift up ！！！！
    rospy.loginfo('==== Lift Up ({:.2f}) ==== \n'.format(saftz))
    res = ur_control.set_speed_slider(0.01)
    waypoints = []
    # wpose = ur_control.group.get_current_pose().pose
    waypoints.append(copy.deepcopy(stop_pos))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
    ur_control.group.execute(plan, wait=True)

    res = ur_control.set_speed_slider(0.4)
    waypoints = []
    stop_pos.position.z = saftz
    waypoints.append(copy.deepcopy(stop_pos))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
    ur_control.group.execute(plan, wait=True)   
    return True

def saftyCheckHard(lift_z,pene_z,safe_fd):
    checkLs = []
    LIFT_Z_MIN = 0.08 # 8 cm
    LIFT_Z_MAX = 0.20 # 20 cm
    PENE_Z_MIN = 0.   # 0 cm
    PENE_Z_MAX = 0.06 # 6 cm
    FORCE_MAX  = 15 # 15 N

    ## check sign
    if np.sign(lift_z) == 1:
        checkLs.append(True)
    else: 
        return False
    
    ## check the lift-up height bounds
    if round(lift_z,6) >= LIFT_Z_MIN and round(lift_z,6) <= LIFT_Z_MAX:
        checkLs.append(True)
    else: 
        return False
    ## check the penetration depth bounds
    if np.sign(pene_z) == 1 and np.abs(round(pene_z,6)) <= LIFT_Z_MAX:
        checkLs.append(True)
    elif np.sign(pene_z) != 1 and np.abs(round(pene_z,6)) <= PENE_Z_MAX:
        checkLs.append(True)
    else: 
        return False
    
    ## check the safe force threshold
    if np.sign(safe_fd) == 1 and np.abs(round(safe_fd,6)) <= FORCE_MAX:
        checkLs.append(True)
    else: 
        return False
    
    ## CAN add other checking items

    ## General safety: return safe if all items are True
    if all(checkLs):
        return True
    else: 
        return False


if __name__ == '__main__':
    rospy.init_node("test_move")
    moveit_commander.roscpp_initialize(sys.argv)
    # # ur_control.speedl_control([0, -0.1, 0, 0, 0, 0], 0.5, 2)

    ################################################################
    ############# ur control #############
    # ur_control = MoveGroupPythonInteface(sim=True)  #simu
    ur_control = MoveGroupPythonInteface(sim=False)  #real
    rospy.loginfo('init ok.')

    ur_control.group.get_planning_frame()
    ur_control.group.get_end_effector_link()

    ur_control.remove_allobjects()
    res = ur_control.play_program()
    rospy.loginfo("play_program: {}".format(res))
    rospy.sleep(1)
    ##obtain the current pose list
    # rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
    # rospy.loginfo('current joint: \n{}'.format(ur_control.get_joint_pos_list()))

    # # set the initial pos (i.e., origin of task frame)
    initPtx = ur_control.group.get_current_pose().pose.position.x
    initPty = ur_control.group.get_current_pose().pose.position.y
    initPtz = ur_control.group.get_current_pose().pose.position.z # surface plane
    initPtx = -0.5931848696000094
    initPty = -0.28895797651231064
    initPtz = 0.07731254732208744 
    # [one option] 
    # initPtx = 
    # initPty = 
    # initPtz =

    # x positive/ x negative/ y positive/ y negative
    xp = 0.25
    xn = 0.0
    yp = 0.35
    yn = 0.0
    
    # limit of x,y (in world frame)
    xmax = initPtx + xp
    xmin = initPtx - xn
    ymax = initPty + yp
    ymin = initPty - yn

    lim = [xmin, xmax, ymin, ymax]
    ## set zero to the ft 300
    zero_ft_sensor()
    

    Lrang = 0.3 # <<<<<<
    ## position of buried objects
    ds_obj = 0.27
    
    LIFT_HEIGHT = +0.10 #(default: +0.10) # <<<<<<
    saftz = initPtz + LIFT_HEIGHT
    # PENETRATION DEPTH
    PENE_DEPTH = 0.05 #(default: -0.03) # <<<<<<
    depthz = initPtz + PENE_DEPTH
    # SAFE FORCE
    SAFE_FORCE = 10.0  #(default: 15N) # <<<<<<
    flargeFlag = 0
    # folder name
    fd_name = 'ur_spiral_traj/data/' # <<<<<<
    #fig_dir = '/home/zhangzeqing/Nutstore Files/Nutstore/ur_spiral_traj/fig'
    isSaveForce = 1           # <<<<<<
    # velocity limits setting
    maxVelScale    = 0.3 # <<<<<<
    normalVelScale = 0.1 # <<<<<<
    ite_bar = 30
    delta_ite = 10
    ds_min = 0.005

    listener = listener()
    ur_control.set_speed_slider(maxVelScale)

    ## check exp safety setting at the beginning
    if saftyCheckHard(LIFT_HEIGHT,PENE_DEPTH,SAFE_FORCE):
        print('***** Safety Check Successfully *****')
    else:
        print('!!!!! Safety Check Failed !!!!!')
        sys.exit(1)
    
    ## go the initial position
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z = saftz
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.x = initPtx
    wpose.position.y = initPty
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z = initPtz    
    quater_init = tfs.quaternion_from_euler(0, np.pi, np.pi/2,'szyz')
    wpose.orientation.x = quater_init[0]
    wpose.orientation.y = quater_init[1]
    wpose.orientation.z = quater_init[2]
    wpose.orientation.w = quater_init[3]
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
    ur_control.group.execute(plan, wait=True)
    print('***** Exp Initialized Successfully *****')
    rospy.sleep(0.5)
    
    ## start the loop
    for j in range(1,21): # <<<<<<
        #region： #experiment of 1 strokes    
        for i in range(1,2): #<<<<<<
            # current angle (deg)
            theta_cur = 90

            ## list to record the df, dr, ds
            df_ls = []
            dr_ls = []
            ds_ls = []            

            # start
            x_s_wldf = initPtx + 0.1
            y_s_wldf = initPty

            # goal (cannot approach since the buried object)
            x_e_wldf = x_s_wldf
            y_e_wldf = y_s_wldf + Lrang   

            ## lift up
            ur_control.set_speed_slider(maxVelScale)
            waypoints = []
            wpose = ur_control.group.get_current_pose().pose
            wpose.position.z = saftz
            waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
            ur_control.group.execute(plan, wait=True)
            
            ## move to the start point (speed up)         
            wpose.position.x = x_s_wldf
            wpose.position.y = y_s_wldf
            waypoints.append(copy.deepcopy(wpose))            
            (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
            ur_control.group.execute(plan, wait=True)   

            # waypoints = []
            # wpose = ur_control.group.get_current_pose().pose
            # ## spiral trajectory
            # ### (r,theta): r = a+b*theta
            # xx = []
            # yy = []
            # a = 0
            # b = 0.01/(2*np.pi) # 360 deg for moving left 1cm distance 
            # cent = [wpose.position.x, wpose.position.y]
            # init_angle = np.pi/2
            # end_angle = 6*np.pi + np.pi
            # # 10 steps for 180 deg
            # num_anlge = int(10*(end_angle/(np.pi)))
            # for theta in np.linspace(0,end_angle,num_anlge):
            #     r = a+b*theta
            #     # xx.append(cent[0]+r*np.cos(theta+init_angle))
            #     # yy.append(cent[1]+r*np.sin(theta+init_angle))                
            #     wpose.position.x = cent[0]+r*np.cos(theta+init_angle)
            #     wpose.position.y = cent[1]+r*np.sin(theta+init_angle)
            #     waypoints.append(copy.deepcopy(wpose))
            # (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
            # ur_control.group.execute(plan, wait=True)
            ur_spiralTraj(ur_control)
                

            ## check the coorrdinate limit
            if checkCoorLimit([x_e_wldf, y_e_wldf], lim):
                ## penetration
                waypoints = []
                wpose.position.z = depthz
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
                ur_control.group.execute(plan, wait=True)
                ur_control.group.stop()
                rospy.sleep(2)                

                ur_control.set_speed_slider(normalVelScale)
                # go to the goal
                waypoints = []
                wpose.position.x = x_e_wldf
                wpose.position.y = y_e_wldf
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
                listener.clear_finish_flag()
                zero_ft_sensor()
                ur_control.group.execute(plan, wait=False)
                rospy.loginfo('clear_finish_flag')
                while not listener.read_finish_flag():                    
                    ## measure the force val/dir
                    f_val = listener.get_force_val()
                    f_dir = listener.get_force_dir()
                    if f_val is not None:
                        ## path distance
                        cur_pos = ur_control.group.get_current_pose().pose
                        curx = cur_pos.position.x
                        cury = cur_pos.position.y
                        dist = round(np.abs(cury - y_s_wldf),6)
                        
                        df_ls.append(round(f_val,6))
                        dr_ls.append(round(f_dir,6))
                        ds_ls.append(dist)
                        # rospy.loginfo('ForceVal (N): {}'.format(f_val))
                        # rospy.loginfo('Distance (m): {}'.format(dist))

                        ## log (internal)
                        # if isSaveForce ==  1:
                        #     ## start to record the data from Ft300
                        #     with open('/home/zhangzeqing/Nutstore Files/Nutstore/{}/znv_exp{}.csv'.format(fd_name,1*(j-1)+i),'a',newline="\n")as f:
                        #         f_csv = csv.writer(f) # <<<<<<
                        #         f_csv.writerow([np.round(f_val,6), np.round(f_dir,6), np.round(dist,6)])
                        #     f.close()  

                        ## most conservative way (most safe)
                        if np.round(f_val,6) > SAFE_FORCE:
                            rospy.loginfo('==== Large Force Warning ==== \n')
                            ur_control.group.stop()
                            flargeFlag = True
                            break
                ## log (external)
                if isSaveForce ==  1:
                    allData = zip(df_ls,dr_ls,ds_ls)
                    ## start to record the data from Ft300
                    with open('/home/zhangzeqing/Nutstore Files/Nutstore/{}/znv_exp{}.csv'.format(fd_name,1*(j-1)+i),'a',newline="\n")as f:
                        f_csv = csv.writer(f) # <<<<<<
                        for row in allData:
                            f_csv.writerow(row)
                    f.close()              
                # rospy.loginfo('Ref Dir(deg): {}'.format(theta_cur))
                # rospy.loginfo('{}-th loop finished'.format(i))
            else:
                rospy.loginfo('Out of the Worksapce:\n x {}, y {}'.format(round(x_e_wldf,3),round(y_e_wldf,3)))
                ur_control.group.stop()
                sys.exit(1)
            if flargeFlag == 1:
                break
        rospy.loginfo('Exp {} finished'.format(j))

    # lift up
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z = saftz
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
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
