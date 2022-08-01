# exp: ur5 + spiral traj + BOA
# [Not Done]
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
# from sympy import re
import rospy
# import tf
# import tf2_ros
from geometry_msgs.msg import PoseArray, TransformStamped
from std_msgs.msg import String

from tf import transformations as tfs
from functions.scene_helper import zero_ft_sensor,ft_listener
from functions.ur_move import MoveGroupPythonInteface,goPeneGivenPose,go2GivenPose,go2GivenPose2
from robotiq_ft_sensor.msg import ft_sensor
from control_msgs.msg import FollowJointTrajectoryActionResult as rlst
import moveit_commander
import sys
import csv
from functions.jamming_detector import JDLib
from functions.handle_drag_force import smooth_fd_kf, get_mean
from functions.drawTraj import urCentOLine,urCentOLine_sim,urCent2Circle
from functions.saftyCheck import saftyCheckHard
from functions.saftyCheck import SfatyPara
from bayes_opt import BayesianOptimization, UtilityFunction
from sklearn.gaussian_process.kernels import RBF,Matern
from functions.boa_helper import plot_2d2

if __name__ == '__main__':
    rospy.init_node("test_move")
    moveit_commander.roscpp_initialize(sys.argv)

    ## input the exp mode
    input = int(input("Exp Mode: trial(0), normal(1) ") or "0")
    if input == 0:
        exp_mode = 'trial'
    elif input == 1:
        exp_mode = 'normal'
    else:
        raise Exception('Error: Invalid Exp Mode!')
    
    ## set the initial pos (i.e., origin of task frame)
    sp = SfatyPara()
    originx = sp.originX
    originy = sp.originY
    originz = sp.originZ

    ##--- BOA related codes ---#
    # kernel = RBF(length_scale=8, length_scale_bounds='fixed')
    # kernel = Matern(length_scale=1, length_scale_bounds='fixed',nu=np.inf)
    lenScaleBound ='fixed'
    # lenScaleBound = (1e-5, 1e5)
    # lenScaleBound = (0.01, 0.2)
    kernel = Matern(length_scale=0.04, length_scale_bounds=lenScaleBound, nu=np.inf)
    # kernel = Matern(length_scale=0.04, length_scale_bounds=lenScaleBound, nu=2.5)
    # kernel = Matern(length_scale=0.04, length_scale_bounds=lenScaleBound, nu=1.5)
    str_kernel = str(kernel)

    ## initial distribution in BOA
    xrange = np.linspace(sp.xmin, sp.xmax, 125)
    yrange = np.linspace(sp.ymin, sp.ymax, 175)
    X, Y = np.meshgrid(xrange, yrange)
    xrange = X.ravel()
    yrange = Y.ravel()
    XY = np.vstack([xrange, yrange]).T
    ##=== BOA related codes ===#

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

    ## set zero to the ft 300
    zero_ft_sensor()    

    Lrang = 0.3 # <<<<<<
    ## position of buried objects
    ds_obj = 0.24
    
    LIFT_HEIGHT = +0.10 #(default: +0.10) # <<<<<<
    # saftz = initPtz + LIFT_HEIGHT
    # PENETRATION DEPTH
    if exp_mode == 'trial':
        PENE_DEPTH = 0.05    #(default: -0.03) # <<<<<<
        normalVelScale = 0.5 # <<<<<<
    elif exp_mode == 'normal':
        PENE_DEPTH = -0.05   #(default: -0.03) # <<<<<<
        normalVelScale = 0.1 # <<<<<<
    else:
        raise Exception('Error: Invalid Exp Mode!')    
    depthz = originz + PENE_DEPTH
    maxVelScale    = 0.3 # <<<<<<
    # Cur SAFE FORCE
    CUR_SAFE_FORCE = 7.0  #(default: 15N) # <<<<<<
    
    # folder name
    expFolderName = '/20220727ur_BOA_spiral' # <<<<<<
    NutStorePath = '/home/zhangzeqing/Nutstore Files/Nutstore'
    dataPath = NutStorePath+expFolderName+'/data'
    figPath = NutStorePath+expFolderName+'/fig'
    isSaveForce = 1           # <<<<<<
    isPlotJD = 1
    ## JD setting
    ite_bar = 30
    delta_ite = 10
    ds_min = 0.005
    JDid = 1
    diff_bar = 0.5 # N # <<<<<<

    listener = ft_listener()
    ## BOA init. (bounds in the relatvie frame)
    bo = BayesianOptimization(f=None, pbounds={'x': (sp.xmin, sp.xmax), 'y': (sp.ymin, sp.ymax)},
                        verbose=2,
                        random_state=1)
    # plt.ioff()
    bo.set_gp_params(kernel=kernel)
    util = UtilityFunction(kind="ei", 
                        kappa = 2, 
                        xi=0.5,
                        kappa_decay=1,
                        kappa_decay_delay=0)
    
    ## velocity setting
    ur_control.set_speed_slider(maxVelScale)

    ## check exp safety setting at the beginning
    if saftyCheckHard(LIFT_HEIGHT,PENE_DEPTH,CUR_SAFE_FORCE):
        print('***** Safety Check Successfully *****')
    else:
        raise Exception('Error: Safety Check Failed')
    
    ## go the origin
    # go2Origin(ur_control)

    ## go the init pos of the exp 
    pose = [0 for hh in range(0,3)]
    pose[0] = originx + 0.1
    pose[1] = originy 
    pose[2] = sp.SAFEZ 
    go2GivenPose(ur_control,pose)
    rospy.sleep(0.5)

    # penetration
    pose[2] = depthz
    go2GivenPose(ur_control,pose)
    rospy.sleep(0.5)
    # zero_ft_sensor()
    fd_nonjamming = 3  # 3N
    traj_radius = 0.01 # xx cm

    ## start the loop
    for slide_id in range(1,21): # <<<<<<
        print("--------- {}-th slide ---------".format(slide_id))
        ## record the start x,y (i.e., current pos) in UR frame
        wpose = ur_control.group.get_current_pose().pose
        x_s_wldf = wpose.position.x
        y_s_wldf = wpose.position.y

        ######### cal. goal by BOA #########        
        # BOA provides the relative goal
        nextPt = bo.suggest(util) # dict type
        # rospy.loginfo(nextPt)
        ex = list(nextPt.values())[0]
        ey = list(nextPt.values())[1]
        print('relative goal x {:.3f}, y {:.3f}'.format(ex,ey))
        # goal in the UR base frame
        x_e_wldf = originx + ex
        y_e_wldf = originy + ey

        ## list to record the df, dr, ds
        df_ls = []
        dr_ls = []
        ds_ls = []    
        ds_ite_ls = []
        maxForward_ls = []  
        rela_x_ls = [] # relative x 
        rela_y_ls = []
        boa_ite_ls = []
        boa_x_ls = []
        boa_y_ls = []
        boa_return_ls = []

        ## initialize parameters for each slide        
        ite = 1
        cent_dist = 0
        
        ## goal
        # x_e_wldf = initPtx + 0.12
        # y_e_wldf = initPty + 0.3

        ## lift up
        # ur_control.set_speed_slider(maxVelScale)
        # waypoints = []
        # wpose = ur_control.group.get_current_pose().pose
        # wpose.position.z = sp.SAFEZ
        # waypoints.append(copy.deepcopy(wpose))
        # (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
        # ur_control.group.execute(plan, wait=True)
        
        # ## move to the start point (speed up)         
        # wpose.position.x = x_s_wldf
        # wpose.position.y = y_s_wldf
        # waypoints.append(copy.deepcopy(wpose))            
        # (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
        # ur_control.group.execute(plan, wait=True)
        
        ## check the coorrdinate limit
        if not sp.checkCoorLimitXY([x_e_wldf, y_e_wldf]):
            rospy.loginfo('Out of the Worksapce:\n x {}, y {}'.format(round(x_e_wldf,3),round(y_e_wldf,3)))
            ur_control.group.stop()
            raise Exception('Out of the Worksapce:\n x {}, y {}'.format(round(x_e_wldf,3),round(y_e_wldf,3)))
        
        ## penetration
        # waypoints = []
        # wpose.position.z = depthz
        # waypoints.append(copy.deepcopy(wpose))
        # (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
        # ur_control.group.execute(plan, wait=True)
        # ur_control.group.stop()
        # rospy.sleep(2)                          

        vect2goalx = x_e_wldf - x_s_wldf
        vect2goaly = y_e_wldf - y_s_wldf
        norm_vect2goal = np.sqrt(vect2goalx**2+vect2goaly**2)
        print('path length: {:.3f}'.format(norm_vect2goal))
        
        ## circle+line (a.k.a. spiral traj.)
        # _,_,waypts = urCentOLine(ur_control,0.01,0.01,[x_e_wldf,y_e_wldf])
        _,_,waypts = urCentOLine_sim(ur_control,traj_radius,0.01,[x_e_wldf,y_e_wldf])
        # _,_,Ocent,waypts = urCent2Circle(ur_control,traj_radius,1,False)
        (plan, fraction) = ur_control.go_cartesian_path(waypts,execute=False)
        ## move along the generated path
        listener.clear_finish_flag()
        ur_control.set_speed_slider(normalVelScale)
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
        flargeFlag = False
        pre_forward_dist = 0.0 
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
                
                ## log list
                cur_pos = ur_control.group.get_current_pose().pose
                curx = cur_pos.position.x
                cury = cur_pos.position.y
                
                vect2curx = curx - x_s_wldf
                vect2cury = cury - y_s_wldf
                norm_vect2cur = np.sqrt(vect2curx**2+vect2cury**2)
                # temp1 = round(vect2goalx*vect2curx + vect2goaly*vect2cury,6)
                # temp2 = round(norm_vect2goal*norm_vect2cur,6)
                temp1 = vect2goalx*vect2curx + vect2goaly*vect2cury
                temp2 = norm_vect2goal*norm_vect2cur
                temp3 = round(abs(temp1 - temp2),6)
                forward_dist = round(norm_vect2cur,3) # lie in [startpos, goal]                                        
                if temp3 <=  1e-07 and forward_dist > 0.001 and forward_dist - pre_forward_dist >0:
                    # print(temp3)
                    # dist = round(norm_vect2cur-traj_radius,4) # x.x mm                        
                    cent_dist = round(forward_dist - traj_radius,3)
                    ds_ls.append(cent_dist)
                    ds_ite_ls.append(ite)
                    print('----center dist {:.3f}----'.format(cent_dist))
                    print('----remaining path: {:.3f} m'.format(norm_vect2goal - cent_dist))
                    pre_forward_dist = forward_dist
                    # tell BOA the observed value
                    boax = curx - originx
                    boay = cury - originy
                    probePt_dict = {'x':boax,'y':boay}                        
                    bo.register(params=probePt_dict, target=fd_nonjamming)

                    boa_ite_ls.append(ite)
                    boa_x_ls.append(round(boax,4))
                    boa_y_ls.append(round(boay,4))
                    boa_return_ls.append(fd_nonjamming)
                
                df_ls.append(round(f_val,4))
                dr_ls.append(round(f_dir,4))
                rela_x_ls.append(round(curx - originx,4))
                rela_y_ls.append(round(cury - originy,4))                    
                
                ite = ite+1

        ## if get contact, tell to the BOA
        if flargeFlag == True:
            curx = ur_control.group.get_current_pose().pose.position.x
            cury = ur_control.group.get_current_pose().pose.position.y
            ## return values into BOA
            boax = curx - originx
            boay = cury - originy
            probePt_dict = {'x':boax,'y':boay}
            # tell BOA the observed value
            bo.register(params=probePt_dict, target=f_val)

            df_ls.append(round(f_val,4))
            dr_ls.append(round(f_dir,4))
            rela_x_ls.append(round(curx - originx,4))
            rela_y_ls.append(round(cury - originy,4))
            boa_ite_ls.append(ite)
            boa_x_ls.append(round(boax,4))
            boa_y_ls.append(round(boay,4))
            boa_return_ls.append(round(f_val,4))

        ## if get contact then start from the goal to the start pos.
        if flargeFlag == True:
            ## record the contact pos.(absolute)
            contactPosX = curx
            contactPosY = cury

            ## flip the start pt and goal in UR frame
            x_ss_wldf = x_e_wldf
            y_ss_wldf = y_e_wldf
            x_ee_wldf = x_s_wldf
            y_ee_wldf = y_s_wldf  
            vect2goalx = x_ee_wldf - x_ss_wldf
            vect2goaly = y_ee_wldf - y_ss_wldf
            norm_vect2goal = np.sqrt(vect2goalx**2+vect2goaly**2)
            norm_vect2cont = np.sqrt((contactPosX-x_ss_wldf)**2+(contactPosY-y_ss_wldf)**2)
            ## if the remaining path shorter than 1cm, go back to init pos
            if round(norm_vect2cont - 0.01,3) <= 0:
                ## go to & penetrate into the initial pos of exp.
                go2GivenPose(ur_control,[originx + 0.1,originy,depthz])
                rospy.sleep(0.5)
                continue
            
            ## try to move to the start pt with penetration
            # pose = [0 for hh in range(0,3)]
            # pose[0] = x_ss_wldf
            # pose[1] = y_ss_wldf
            # pose[2] = depthz
            # go2GivenPose(ur_control,pose)
            
            # go2GivenPose2(ur_control,pose,normalVelScale)
            flag1 = goPeneGivenPose(ur_control,[x_ss_wldf,y_ss_wldf,depthz],normalVelScale)
            
            if flag1 == True:
                ## can penetrate into the start pt (i.e., previous goal)

                # ## flip the start and goal in UR frame
                # x_ss_wldf = x_e_wldf
                # y_ss_wldf = y_e_wldf
                # # goal in the UR base frame
                # x_ee_wldf = x_s_wldf
                # y_ee_wldf = y_s_wldf

                ## sprial traj. to the previous start position
                _,_,waypts = urCentOLine_sim(ur_control,traj_radius,0.01,[x_ee_wldf,y_ee_wldf])
                # _,_,Ocent,waypts = urCent2Circle(ur_control,traj_radius,1,False)
                (plan, fraction) = ur_control.go_cartesian_path(waypts,execute=False)
                ## move along the generated path
                listener.clear_finish_flag()
                ur_control.set_speed_slider(normalVelScale)
                zero_ft_sensor()
                ur_control.group.execute(plan, wait=False)

                ## --- [force monitor] ---
                rospy.loginfo('clear_finish_flag')
                pre_forward_dist = 0.0 
                while not listener.read_finish_flag():                    
                    ## measure the force val/dir
                    f_val = listener.get_force_val()
                    f_dir = listener.get_force_dir()
                    if f_val is not None:
                        ## most conservative way (most safe)
                        if np.round(f_val,6) > CUR_SAFE_FORCE:
                            rospy.loginfo('==== Large Force Warning ==== \n')
                            ur_control.group.stop()  
                            # contactFlag = True
                            ## must get contact again                            
                            ## get contact & tell to the BOA
                            curx = ur_control.group.get_current_pose().pose.position.x
                            cury = ur_control.group.get_current_pose().pose.position.y
                            probePt_dict = {'x':curx - originx,'y':cury - originy}
                            bo.register(params=probePt_dict, target=f_val)
                            ## lift up and move back to & penetrate into the start pt (i.e., previous goal)
                            flag2 = goPeneGivenPose(ur_control,[x_ss_wldf,y_ss_wldf,depthz],normalVelScale)
                            if flag2 == False: raise Exception('Err: something unexpected')
                            break               
                        
                        ## log list
                        cur_pos = ur_control.group.get_current_pose().pose
                        curx = cur_pos.position.x
                        cury = cur_pos.position.y
                        vect2curx = curx - x_ss_wldf
                        vect2cury = cury - y_ss_wldf
                        norm_vect2cur = np.sqrt(vect2curx**2+vect2cury**2)
                        temp1 = vect2goalx*vect2curx + vect2goaly*vect2cury
                        temp2 = norm_vect2goal*norm_vect2cur
                        temp3 = round(abs(temp1 - temp2),6)
                        forward_dist = round(norm_vect2cur,3) # lie in [startpos, goal] 
                        # print('temp3: {:.3f}'.format(temp3))
                        # print('forward_dist: {:.3f}'.format(forward_dist))                                       
                        if temp3 <=  1e-07 and forward_dist > 0.001 and forward_dist - pre_forward_dist >0:
                            # print(temp3)
                            # dist = round(norm_vect2cur-traj_radius,4) # x.x mm                        
                            cent_dist = round(forward_dist - traj_radius,3)
                            ds_ls.append(cent_dist)
                            ds_ite_ls.append(ite)
                            print('----center dist {:.3f}----'.format(cent_dist))
                            pre_forward_dist = forward_dist
                            # tell BOA the observed value
                            boax = curx - originx
                            boay = cury - originy
                            probePt_dict = {'x':boax,'y':boay}                        
                            bo.register(params=probePt_dict, target=fd_nonjamming)

                            boa_ite_ls.append(ite)
                            boa_x_ls.append(round(boax,4))
                            boa_y_ls.append(round(boay,4))
                            boa_return_ls.append(fd_nonjamming)
                        
                        df_ls.append(round(f_val,4))
                        dr_ls.append(round(f_dir,4))
                        rela_x_ls.append(round(curx - originx,4))
                        rela_y_ls.append(round(cury - originy,4))                    
                        
                        ite = ite+1
            else:
                ## cannot penetrate into the granular media at the start point
                ### tell to the BOA that the start pt has high force (e.g., 7N)
                boax = x_ss_wldf - originx
                boay = y_ss_wldf - originy
                probePt_dict = {'x':boax,'y':boay}
                bo.register(params=probePt_dict, target=7)
                ### penetrate step by step until to the previous contact pos.
                jj = 1
                step_len = 0.01 # 1cm
                flag3 = True
                while round(jj*step_len-norm_vect2cont,6) <= 0:                
                    new_startPtX = x_ss_wldf + (jj*step_len/norm_vect2goal)*vect2goalx
                    new_startPtY = y_ss_wldf + (jj*step_len/norm_vect2goal)*vect2goaly
                    flag3 = goPeneGivenPose(ur_control,[new_startPtX,new_startPtY,depthz],normalVelScale)
                    ## ---- if can not penetrate ----
                    if flag3 == False:
                        ### tell to the BOA that the NEW start pt has high force (e.g., 7N)
                        boax = new_startPtX - originx
                        boay = new_startPtY - originy
                        probePt_dict = {'x':boax,'y':boay}
                        bo.register(params=probePt_dict, target=7)
                        jj = jj+1
                        continue
                    ## ---- if can penetrate ----
                    ### sprial traj. to the previous start position
                    _,_,waypts = urCentOLine_sim(ur_control,traj_radius,0.01,[x_ee_wldf,y_ee_wldf])
                    # _,_,Ocent,waypts = urCent2Circle(ur_control,traj_radius,1,False)
                    (plan, fraction) = ur_control.go_cartesian_path(waypts,execute=False)
                    ### move along the generated path
                    listener.clear_finish_flag()
                    ur_control.set_speed_slider(normalVelScale)
                    zero_ft_sensor()
                    ur_control.group.execute(plan, wait=False)

                    ### --- [force monitor] ---
                    rospy.loginfo('clear_finish_flag')
                    contactFlag = False
                    pre_forward_dist = 0.0 
                    while not listener.read_finish_flag():                    
                        ## measure the force val/dir
                        f_val = listener.get_force_val()
                        f_dir = listener.get_force_dir()
                        if f_val is not None:
                            ### most conservative way (most safe)
                            if np.round(f_val,6) > CUR_SAFE_FORCE:
                                ## must get contact again
                                rospy.loginfo('==== Large Force Warning ==== \n')
                                ur_control.group.stop()
                                contactFlag = True                                                                
                                ## get contact & tell to the BOA
                                curx = ur_control.group.get_current_pose().pose.position.x
                                cury = ur_control.group.get_current_pose().pose.position.y
                                probePt_dict = {'x':curx - originx,'y':cury - originy}
                                bo.register(params=probePt_dict, target=f_val)
                                ## lift up and move to & penetrate into the start pt
                                flag4 = goPeneGivenPose(ur_control,[x_ss_wldf,y_ss_wldf,depthz],normalVelScale)
                                if flag4 == False: raise Exception('Err: something unexpected')
                                break                    
                            
                            ### log list
                            cur_pos = ur_control.group.get_current_pose().pose
                            curx = cur_pos.position.x
                            cury = cur_pos.position.y
                            vect2curx = curx - x_ss_wldf
                            vect2cury = cury - y_ss_wldf
                            norm_vect2cur = np.sqrt(vect2curx**2+vect2cury**2)
                            temp1 = vect2goalx*vect2curx + vect2goaly*vect2cury
                            temp2 = norm_vect2goal*norm_vect2cur
                            temp3 = round(abs(temp1 - temp2),6)
                            forward_dist = round(norm_vect2cur,3) # lie in [startpos, goal] 
                            # print('temp3: {:.3f}'.format(temp3))
                            # print('forward_dist: {:.3f}'.format(forward_dist))                                       
                            if temp3 <=  1e-07 and forward_dist > 0.001 and forward_dist - pre_forward_dist >0:
                                # print(temp3)
                                # dist = round(norm_vect2cur-traj_radius,4) # x.x mm                        
                                cent_dist = round(forward_dist - traj_radius,3)
                                ds_ls.append(cent_dist)
                                ds_ite_ls.append(ite)
                                print('----center dist {:.3f}----'.format(cent_dist))
                                pre_forward_dist = forward_dist
                                ### tell BOA the observed value
                                boax = curx - originx
                                boay = cury - originy
                                probePt_dict = {'x':boax,'y':boay}                        
                                bo.register(params=probePt_dict, target=fd_nonjamming)

                                boa_ite_ls.append(ite)
                                boa_x_ls.append(round(boax,4))
                                boa_y_ls.append(round(boay,4))
                                boa_return_ls.append(fd_nonjamming)
                            
                            df_ls.append(round(f_val,4))
                            dr_ls.append(round(f_dir,4))
                            rela_x_ls.append(round(curx - originx,4))
                            rela_y_ls.append(round(cury - originy,4))                    
                            
                            ite = ite+1
                    if contactFlag == True:
                        break
                    else:
                        ## must can get contact
                        raise Exception('Err: something unexpected')
                if flag3 == False:
                    ## go to & penetrate into the initial pos of exp.
                    go2GivenPose(ur_control,[originx + 0.1,originy,depthz])
                    rospy.sleep(0.5)

        ## if get contact, tell to the BOA
        # if flargeFlag == True:
        #     curx = ur_control.group.get_current_pose().pose.position.x
        #     cury = ur_control.group.get_current_pose().pose.position.y
        #     ## return values into BOA
        #     boax = curx - originx
        #     boay = cury - originy
        #     probePt_dict = {'x':boax,'y':boay}
        #     # tell BOA the observed value
        #     bo.register(params=probePt_dict, target=f_val)

        #     df_ls.append(round(f_val,4))
        #     dr_ls.append(round(f_dir,4))
        #     rela_x_ls.append(round(curx - originx,4))
        #     rela_y_ls.append(round(cury - originy,4))
        #     boa_ite_ls.append(ite)
        #     boa_x_ls.append(round(boax,4))
        #     boa_y_ls.append(round(boay,4))
        #     boa_return_ls.append(round(f_val,4))
        
        ## log (external)
        if isSaveForce ==  1:
            now_date = time.strftime("%m%d%H%M%S", time.localtime())
            allData = zip(rela_x_ls,rela_y_ls,df_ls,dr_ls)
            ## log: x_rela, y_rela, force val, force dir                
            with open('{}/{}_slide{}_Fdvaldir.csv'.format(dataPath,now_date,slide_id),'a',newline="\n")as f:
                f_csv = csv.writer(f) # <<<<<<
                for row in allData:
                    f_csv.writerow(row)
            f.close()
            ## log: ite - center distance
            allData = zip(ds_ite_ls,ds_ls)
            with open('{}/{}_slide{}_Distance.csv'.format(dataPath,now_date,slide_id),'a',newline="\n")as f:
                f_csv = csv.writer(f) # <<<<<<
                for row in allData:
                    f_csv.writerow(row)
            f.close()

            ## log: 4 info. on BOA                
            allData = zip(boa_ite_ls,boa_x_ls,boa_y_ls,boa_return_ls)
            with open('{}/{}_slide{}_BOA.csv'.format(dataPath,now_date,slide_id),'a',newline="\n")as f:
                f_csv = csv.writer(f) # <<<<<<
                ## record the start and goal (relative)
                tempRow = [x_s_wldf-originx, y_s_wldf-originy, x_e_wldf-originx, y_e_wldf-originy]
                f_csv.writerow(tempRow)
                for row in allData:
                    f_csv.writerow(row)
            f.close()

        ## if no jamming, plot it， and ds_ls not empty
        # if isPlotJD and not flargeFlag and ds_ls:
        #         ds_adv = round(ds_obj-ds_ls[-1], 3) # >0 in theory
        #         title_str = 'Exp{}: ds [{},{}], Dep {}, Vel {}, Ite {}, NoJD'.format(slide_id,ds_min,np.inf,PENE_DEPTH,normalVelScale,len(df_ls))
        #         JDlib.plotJDRes(ds_obj,title_str,figPath,slide_id)

        ## plot BOA results
        ## Def: plot_2d2(slide_id, bo, util, kernel,x,y,XY, f_max, fig_path, name=None)
        plot_2d2(slide_id, bo, util, kernel, xrange,yrange,XY, CUR_SAFE_FORCE, figPath+'/{}_slide{}_'.format(now_date,slide_id), "{:03}".format(len(bo._space.params)))
        
        rospy.loginfo('{}-th slide finished'.format(slide_id))

    # lift up
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z = sp.SAFEZ
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
    ur_control.group.execute(plan, wait=True)    
    # endregion
    
    rospy.loginfo('shut down')
