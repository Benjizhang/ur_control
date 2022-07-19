# UR follows the spiral trajectory
# 
# Z. Zhang
# 2022/7

import numpy as np
import copy

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