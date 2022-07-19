# functions to make sure the safty
# 
# Benji Zhang
# 2022/07

import rospy
import copy
import numpy as np

# check whether in the limit 
def checkCoorLimit(pos, lim):
    curx = pos[0]
    cury = pos[1]
    if curx >= lim[0] and curx <= lim[1] and \
        cury >= lim[2] and cury <= lim[3]:
        return True
    else: return False

# emergency stop
def emergency_stop(ur_control,saftz):
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

def emergency_stop2(ur_control,stop_pos, saftz):
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

# check some important parameters (given hard constraints)
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