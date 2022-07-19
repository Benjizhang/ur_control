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
import csv
import pylab
from jamming_detector import jamming_detector1 as jd1
from handle_drag_force import smooth_fd_kf, get_mean
import robotiq_ft_sensor.srv
from functions.drawTraj import ur_spiralTraj,ur_Otraj
from functions.saftyCheck import checkCoorLimit,saftyCheckHard

class listener():
    def __init__(self):
        
        self.obj_pose_sub = rospy.Subscriber("robotiq_ft_sensor", ft_sensor, self.detect_callbak, queue_size=1)
        self.result_status = rospy.Subscriber("/scaled_pos_joint_traj_controller/follow_joint_trajectory/result",rlst,self.callback, queue_size=1)
        self.lock_read = threading.Lock()
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
    

    Lrang = 0.2 # <<<<<<
    ## position of buried objects
    ds_obj = 0.27
    
    LIFT_HEIGHT = +0.10 #(default: +0.10) # <<<<<<
    saftz = initPtz + LIFT_HEIGHT
    # PENETRATION DEPTH
    PENE_DEPTH = -0.05 #(default: -0.03) # <<<<<<
    depthz = initPtz + PENE_DEPTH
    # SAFE FORCE
    SAFE_FORCE = 10.0  #(default: 15N) # <<<<<<
    flargeFlag = 0
    # folder name
    fd_name = 'ur_spiral_traj/data/' # <<<<<<
    #fig_dir = '/home/zhangzeqing/Nutstore Files/Nutstore/ur_spiral_traj/fig'
    isSaveForce = 0           # <<<<<<
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

                ur_control.set_speed_slider(0.5)
                zero_ft_sensor()
                ## sprial traj.
                # ur_spiralTraj(ur_control)

                ## circle traj.
                ur_Otraj(ur_control)

                # go to the goal
                ur_control.set_speed_slider(normalVelScale)
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
