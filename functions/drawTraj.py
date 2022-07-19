# control ur following different trajectories
# 
# Z. Zhang
# 2022/7

import time
import copy
import numpy as np
from tf import transformations as tfs


def urSpiralTraj(ur_control):
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
    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
    ur_control.group.execute(plan, wait=True)
    time.sleep(0.5)
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

def urOtraj(ur_control):
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
    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
    ur_control.group.execute(plan, wait=True)
    time.sleep(0.5)
    return True

## from current pos (circle center) to the given circle by the spiral trajectory
def urCent2Circle(ur_control,radius,numLoop2circle):    
    # coordinates
    x = []
    y = []

    # the current position as the center of the circle
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    cent = [wpose.position.x, wpose.position.y]

    # para. of spiral trajectory
    a = 0    
    delta_radius = radius/numLoop2circle # 1 loop/360 deg for moving delta_radius distance
    b = delta_radius/(2*np.pi)

    init_angle = np.pi/2
    end_angle = 2*np.pi*numLoop2circle
    # 36 segments for 180 deg
    num_anlge = int(36*(end_angle/(np.pi)))

    # generate spiral traj to the given circle
    for theta in np.linspace(0,end_angle,num_anlge):
        r = a+b*theta
        wpose.position.x = cent[0]+r*np.cos(theta+init_angle)
        wpose.position.y = cent[1]+r*np.sin(theta+init_angle)
        waypoints.append(copy.deepcopy(wpose))

        x.append(wpose.position.x)
        y.append(wpose.position.y)
    
    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
    ur_control.group.execute(plan, wait=True)
    time.sleep(0.5)
    
    return x,y,cent

## from current pos (NOT the circle center) to the next circle with the given center
## by spiral trajectory
def urPt2Circle(ur_control,Ocent,radius,numLoop2circle):
    # coordinates
    x = []
    y = []

    # current pos    
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    curx = wpose.position.x
    cury = wpose.position.y

    # expand / shrink to the circle
    dist2Cent = np.sqrt((Ocent[0]-curx)**2+(Ocent[1]-cury)**2)

    delta_radius = (radius-dist2Cent)/numLoop2circle # 1 loop/360 deg for moving delta_radius distance
    b = delta_radius/(2*np.pi)
    
    init_angle = np.arctan2(cury-Ocent[1],curx-Ocent[0])
    end_angle = 2*np.pi*numLoop2circle
    # 36 segments for 180 deg
    num_anlge = int(36*(end_angle/(np.pi)))

    # generate spiral traj to the given circle
    for theta in np.linspace(0,end_angle,num_anlge):
        r = dist2Cent+b*theta
        wpose.position.x = Ocent[0]+r*np.cos(theta+init_angle)
        wpose.position.y = Ocent[1]+r*np.sin(theta+init_angle)
        waypoints.append(copy.deepcopy(wpose))

        x.append(wpose.position.x)
        y.append(wpose.position.y)
    
    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
    ur_control.group.execute(plan, wait=True)
    time.sleep(0.5)

    return x,y

## keep in given circle
def keepCircle(ur_control,Ocent,numLoop):
    # coordinates
    x = []
    y = []

    # current pos
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    curx = wpose.position.x
    cury = wpose.position.y


    radius = np.sqrt((Ocent[0]-curx)**2+(Ocent[1]-cury)**2)
    if round(radius,3) != 0:
        init_angle = np.arctan2(cury-Ocent[1],curx-Ocent[0])
        end_angle = 2*np.pi*numLoop
        # 36 segments for 180 deg
        num_anlge = int(36*(end_angle/(np.pi)))
        # generate circle
        for theta in np.linspace(0,end_angle,num_anlge):
            wpose.position.x = Ocent[0]+radius*np.cos(theta+init_angle)
            wpose.position.y = Ocent[1]+radius*np.sin(theta+init_angle)
            waypoints.append(copy.deepcopy(wpose))

            x.append(wpose.position.x)
            y.append(wpose.position.y)
        
        (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
        ur_control.group.execute(plan, wait=True)
        time.sleep(0.5)

        return x,y
    else:
        # do nothing
        return curx,cury



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

    (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
    ur_control.group.execute(plan, wait=True)
    ur_control.group.stop()