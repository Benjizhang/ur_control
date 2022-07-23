# exp: ur5 + Kalman Filter + JD + spiral traj.
#
# Penetration Depth: D5cm # <<<<<<
# speed/velocity: 0.1 （slow motion）
# 
# Z. Zhang
# 07/22

from cmath import cos
import copy
import threading
import time

# import apriltag_ros.msg
import numpy as np
from sqlalchemy import true
from sympy import re
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseArray, TransformStamped
from std_msgs.msg import String

from tf import transformations as tfs
from functions.scene_helper import zero_ft_sensor
from functions.ur_move import MoveGroupPythonInteface,go2Origin,go2GivenPose
from robotiq_ft_sensor.msg import ft_sensor
from control_msgs.msg import FollowJointTrajectoryActionResult as rlst
import math
import moveit_commander
import sys
import csv
from functions.jamming_detector import JDLib
# from functions.jamming_detector import jamming_detector1 as jd1
# from functions.jamming_detector import plotJDRes
from functions.handle_drag_force import smooth_fd_kf, get_mean
# import robotiq_ft_sensor.srv
from functions.drawTraj import urSpiralTraj,urOtraj,urCent2Circle,urPt2Circle,keepCircle,urCentOLine
from functions.saftyCheck import saftyCheckHard
from functions.saftyCheck import SfatyPara

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


if __name__ == '__main__':
    rospy.init_node("test_move")
    moveit_commander.roscpp_initialize(sys.argv)
    
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

    ## set the initial pos (i.e., origin of task frame)
    sp = SfatyPara()
    initPtx = sp.originX
    initPty = sp.originY
    initPtz = sp.originZ

    ## set zero to the ft 300
    zero_ft_sensor()
    

    Lrang = 0.3 # <<<<<<
    ## position of buried objects
    ds_obj = 0.24
    
    LIFT_HEIGHT = +0.10 #(default: +0.10) # <<<<<<
    # saftz = initPtz + LIFT_HEIGHT
    # PENETRATION DEPTH
    PENE_DEPTH = -0.05  #(default: -0.03) # <<<<<<
    depthz = initPtz + PENE_DEPTH
    # Cur SAFE FORCE
    CUR_SAFE_FORCE = 10.0  #(default: 15N) # <<<<<<
    flargeFlag = 0
    # folder name
    expFolderName = '/20220720spiralTraj' # <<<<<<
    NutStorePath = '/home/zhangzeqing/Nutstore Files/Nutstore'
    dataPath = NutStorePath+expFolderName+'/data'
    figPath = NutStorePath+expFolderName+'/fig'
    isSaveForce = 1           # <<<<<<
    isPlotJD = 1
    # velocity limits setting
    maxVelScale    = 0.3 # <<<<<<
    normalVelScale = 0.1 # <<<<<<
    ite_bar = 30
    delta_ite = 10
    ds_min = 0.005
    JDid = 1
    diff_bar = 0.5 # N # <<<<<<

    listener = listener()
    ur_control.set_speed_slider(maxVelScale)

    ## check exp safety setting at the beginning
    if saftyCheckHard(LIFT_HEIGHT,PENE_DEPTH,CUR_SAFE_FORCE):
        print('***** Safety Check Successfully *****')
    else:
        raise Exception('Error: Safety Check Failed')
    
    ## go the origin
    # go2Origin(ur_control)

    pose = [0 for x in range(0,3)]
    pose[0] = initPtx + 0.1
    pose[1] = initPty 
    pose[2] = sp.SAFEZ 
    go2GivenPose(ur_control,pose)
    rospy.sleep(2)
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
            wpose.position.z = sp.SAFEZ
            waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
            ur_control.group.execute(plan, wait=True)
            
            ## move to the start point (speed up)         
            wpose.position.x = x_s_wldf
            wpose.position.y = y_s_wldf
            waypoints.append(copy.deepcopy(wpose))            
            (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
            ur_control.group.execute(plan, wait=True)
            
            ## check the coorrdinate limit
            if sp.checkCoorLimitXY([x_e_wldf, y_e_wldf]):
                ## penetration
                waypoints = []
                wpose.position.z = depthz
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
                ur_control.group.execute(plan, wait=True)
                ur_control.group.stop()
                rospy.sleep(2)                          

                # ur_control.set_speed_slider(0.5)
                ur_control.set_speed_slider(normalVelScale)
                zero_ft_sensor()
                                
                ## circle+line
                x,y,waypts = urCentOLine(ur_control,0.01,0.01,[x_e_wldf,y_e_wldf])
                (plan, fraction) = ur_control.go_cartesian_path(waypts,execute=False)
                listener.clear_finish_flag()
                zero_ft_sensor()
                ur_control.group.execute(plan, wait=False)

                # go to the goal (line)
                # ur_control.set_speed_slider(normalVelScale)
                # waypoints = []
                # wpose.position.x = x_e_wldf
                # wpose.position.y = y_e_wldf
                # waypoints.append(copy.deepcopy(wpose))
                # (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
                # listener.clear_finish_flag()
                # zero_ft_sensor()
                # ur_control.group.execute(plan, wait=False)

                ## --- [force monitor] ---
                rospy.loginfo('clear_finish_flag')
                while not listener.read_finish_flag():                    
                    ## measure the force val/dir
                    f_val = listener.get_force_val()
                    f_dir = listener.get_force_dir()
                    if f_val is not None:
                        ## most conservative way (most safe)
                        if np.round(f_val,6) > CUR_SAFE_FORCE:
                            rospy.loginfo('==== Large Force Warning ==== \n')
                            ur_control.group.stop()
                            flargeFlag = True
                            break

                        ## path distance
                        cur_pos = ur_control.group.get_current_pose().pose
                        curx = cur_pos.position.x
                        cury = cur_pos.position.y
                        dist = round(np.abs(cury - y_s_wldf),6)
                        
                        ### only record the stable duration
                        if round(dist - ds_min, 6) >= 0:
                            df_ls.append(round(f_val,6))
                            dr_ls.append(round(f_dir,6))
                            ds_ls.append(dist)
                        
                        ## using jamming detector 1
                        # if len(df_ls) >= ite_bar and len(df_ls)%delta_ite==0:
                        #     ## Kalman Filter
                        #     fdhat, Pminus = smooth_fd_kf(df_ls)
                        #     ## Mean
                        #     fdmean = get_mean(df_ls)
                        #     ## absolute difference (N)
                        #     cur_abs_diff = round(abs(fdhat[-1] - fdmean[-1]),3)
                        #     ## detect jamming
                        #     # Mx, isJamming = jd1(fdhat, fdmean, 0.5, delta_ite) 
                        #     JDlib = JDLib(df_ls,ds_ls,fdhat,fdmean,diff_bar)
                        #     isJamming = JDlib.JD(JDid)
                        #     if isJamming:
                        #         rospy.loginfo('**== Jamming Detected ==** \n')
                                # ## ----- v1.0 -----
                                # ur_control.group.stop()
                                # flargeFlag = True
                                # ## plot jamming result
                                # if isPlotJD:
                                #     ds_adv = round(ds_obj-ds_ls[-1], 3) # >0 in theory
                                #     title_str = 'Exp{}: ds [{},{}], Dep {}, Vel {}, Ite {}, Diff {}, Adv {}'.format(j,ds_min,np.inf,PENE_DEPTH,normalVelScale,len(df_ls),cur_abs_diff,ds_adv)
                                #     JDlib.plotJDRes(ds_obj,title_str,figPath,j)
                                # break
                
                ## log (external)
                if isSaveForce ==  1:
                    allData = zip(df_ls,dr_ls,ds_ls)
                    ## start to record the data from Ft300
                    now_date = time.strftime("%m%d%H%M%S", time.localtime())
                    with open('{}/{}_exp{}.csv'.format(dataPath,now_date,1*(j-1)+i),'a',newline="\n")as f:
                        f_csv = csv.writer(f) # <<<<<<
                        for row in allData:
                            f_csv.writerow(row)
                    f.close()

                ## if no jamming, plot it， and ds_ls not empty
                # if isPlotJD and not flargeFlag and ds_ls:
                #         ds_adv = round(ds_obj-ds_ls[-1], 3) # >0 in theory
                #         title_str = 'Exp{}: ds [{},{}], Dep {}, Vel {}, Ite {}, NoJD'.format(j,ds_min,np.inf,PENE_DEPTH,normalVelScale,len(df_ls))
                #         JDlib.plotJDRes(ds_obj,title_str,figPath,j)

            else:
                rospy.loginfo('Out of the Worksapce:\n x {}, y {}'.format(round(x_e_wldf,3),round(y_e_wldf,3)))
                ur_control.group.stop()
                raise Exception('Out of the Worksapce:\n x {}, y {}'.format(round(x_e_wldf,3),round(y_e_wldf,3)))
            if flargeFlag == 1:
                break
        rospy.loginfo('Exp {} finished'.format(j))

    # lift up
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z = sp.SAFEZ
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
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
