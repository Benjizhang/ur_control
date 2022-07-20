# functions to make sure the safty
# 
# Benji Zhang
# 2022/07

import rospy
import copy
import numpy as np

# class includes the important sfaty hard constraints
class SfatyPara:

    def __init__(self):
        
        # safety force threshold
        self.FORCE_MAX  = 15.   # 15 N
        # lift/penetration limits
        self.LIFT_Z_MIN = +0.08 # +8 cm
        self.LIFT_Z_MAX = +0.20 # +20 cm
        self.PENE_Z_MIN = -0.   # -0 cm
        self.PENE_Z_MAX = -0.06 # -6 cm

        # origin coordinates in the UR base frame
        self.originX = -0.5931848696000094
        self.originY = -0.28895797651231064
        self.originZ = 0.07731254732208744
        # safe box (relative frame)        
        self.xmin = 0.   # 0 cm
        self.xmax = 0.25 # +25 cm
        self.ymin = 0.   # 0 cm
        self.ymax = 0.35 # +25 cm
        self.zmin = self.PENE_Z_MAX # -6 cm
        self.zmax = self.LIFT_Z_MAX # +20cm
        # safe box (UR base frame)        
        self.XMIN = self.originX + self.xmin
        self.XMAX = self.originX + self.xmax
        self.YMIN = self.originY + self.ymin
        self.YMAX = self.originY + self.ymax
        self.ZMIN = self.originZ + self.zmin
        self.ZMAX = self.originZ + self.zmax
        # safety height to translation
        self.SAFEZ = self.originZ + 0.10 # +10cm
    
    # check a 3d position is in the limit or not
    def checkCoorLimit3d(self,pos):
        curx = pos[0]
        cury = pos[1]
        curz = pos[2]
        if curx >= self.XMIN and curx <= self.XMAX and \
           cury >= self.YMIN and cury <= self.YMAX and \
           curz >= self.ZMIN and curz <= self.ZMAX:
            return True
        else: return False

    # check whether in the X-Y limit 
    def checkCoorLimitXY(self,pos):
        curx = pos[0]
        cury = pos[1]
        if curx >= self.XMIN and curx <= self.XMAX and \
            cury >= self.YMIN and cury <= self.YMAX:
            return True
        else: return False

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

# check some important given parameters (v.s. hard constraints)
def saftyCheckHard(lift_z,pene_z,safe_fd):
    checkLs = []
    sp = SfatyPara()
    LIFT_Z_MIN = abs(sp.LIFT_Z_MIN) # 8 cm
    LIFT_Z_MAX = abs(sp.LIFT_Z_MAX) # 20 cm
    PENE_Z_MIN = abs(sp.PENE_Z_MIN) # 0 cm
    PENE_Z_MAX = abs(sp.PENE_Z_MAX) # 6 cm
    FORCE_MAX  = sp.FORCE_MAX  # 15 N

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