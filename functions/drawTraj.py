# control ur following different trajectories
# 
# Z. Zhang
# 2022/7

import copy
import numpy as np
from tf import transformations as tfs


def ur_spiralTraj(ur_control):
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    ## spiral trajectory
    ### (r,theta): r = a+b*theta
    xx = []
    yy = []
    a = 0
    b = 0.01/(2*np.pi) # 360 deg for moving 1cm distance 
    cent = [wpose.position.x, wpose.position.y]
    init_angle = np.pi/2
    end_angle = 6*np.pi + np.pi
    # 10 steps for 180 deg
    num_anlge = int(10*(end_angle/(np.pi)))
    for theta in np.linspace(0,end_angle,num_anlge):
        r = a+b*theta
        # xx.append(cent[0]+r*np.cos(theta+init_angle))
        # yy.append(cent[1]+r*np.sin(theta+init_angle))                
        wpose.position.x = cent[0]+r*np.cos(theta+init_angle)
        wpose.position.y = cent[1]+r*np.sin(theta+init_angle)
        waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
    ur_control.group.execute(plan, wait=True)

    return True

def genOtraj(cent,radius,start_angle,end_angle):
    x = []
    y = []
    # 10 steps for 180 deg
    num_anlge = int(10*(abs(end_angle-start_angle)/(np.pi)))

    for theta in np.linspace(start_angle,end_angle,num_anlge):
        x.append(cent[0]+radius*np.cos(theta))
        y.append(cent[1]+radius*np.sin(theta))
    return x,y

def ur_Otraj(ur_control):
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose

    cent = [wpose.position.x, wpose.position.y]
    radius = 0.02
    start_angle = 0
    end_angle = 6*np.pi + np.pi
    x,y = genOtraj(cent,radius,start_angle,end_angle)

    
    for k in range(len(x)):
        wpose.position.x = x[k]
        wpose.position.y = y[k]
        waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
    ur_control.group.execute(plan, wait=True)

    return True


def move_along_boundary(ur_control,lim):
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