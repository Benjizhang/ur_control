# scirpt about ur_control + BOA
# 
# Z. Zhang
# 06/22

from cmath import cos
import copy
import threading

import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseArray, TransformStamped
from std_msgs.msg import String
from bayes_opt import BayesianOptimization, UtilityFunction
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib import mlab
from matplotlib import gridspec

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

def emergency_stop(saftz):
    rospy.loginfo('==== Emergency Stop ==== \n')

    # quick lift up ！！！！
    rospy.loginfo('==== Lift Up ({:.2f}) ==== \n'.format(saftz))
    res = ur_control.set_speed_slider(0.2)
    waypoints = []
    wpose = ur_control.group.get_current_pose().pose
    wpose.position.z = saftz
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
    ur_control.group.execute(plan, wait=True)   
    return True


##--- BOA related codes ---##

### unique_rows
def unique_rows(a):
    """
    A functions to trim repeated rows that may appear when optimizing.
    This is necessary to avoid the sklearn GP object from breaking

    :param a: array to trim repeated rows from

    :return: mask of unique rows
    """

    # Sort array and kep track of where things should go back to
    order = np.lexsort(a.T)
    reorder = np.argsort(order)

    a = a[order]
    diff = np.diff(a, axis=0)
    ui = np.ones(len(a), 'bool')
    ui[1:] = (diff != 0).any(axis=1)

    return ui[reorder]

### posterior
def posterior(bo, X):
    ur = unique_rows(bo._space.params)
    bo._gp.fit(bo._space.params[ur], bo._space.target[ur])
    mu, sigma2 = bo._gp.predict(X, return_std=True)
    ac = util.utility(X, bo._gp, bo._space.target.max())

    return mu, np.sqrt(sigma2), ac

### plot_2d
def plot_2d(name=None):
    mu, s, ut = posterior(bo, XY)
    fig, ax = plt.subplots(2, 2, figsize=(14, 10))
    gridsize=150

    # fig.suptitle('Bayesian Optimization in Action', fontdict={'size':30})

    # GP regression output
    ax[0][0].set_title('Gausian Process Predicted Mean', fontdict={'size':15})
    im00 = ax[0][0].hexbin(x, y, C=mu, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=zmin, vmax=zmax)
    ax[0][0].axis([x.min(), x.max(), y.min(), y.max()])
    ax[0][0].plot(bo._space.params[:, 0], bo._space.params[:, 1], 'D', markersize=4, color='k', label='Observations')
    ax[0][0].plot(xbd,ybd,'k-', lw=2, color='k')


    ax[0][1].set_title('Target Function', fontdict={'size':15})
    im10 = ax[0][1].hexbin(x, y, C=z, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=zmin, vmax=zmax)
    ax[0][1].axis([x.min(), x.max(), y.min(), y.max()])
    ax[0][1].plot(bo._space.params[:, 0], bo._space.params[:, 1], 'D', markersize=4, color='k')
    ax[0][1].plot(xbd,ybd,'k-', lw=2, color='k')


    ax[1][0].set_title('Gausian Process Variance', fontdict={'size':15})
    im01 = ax[1][0].hexbin(x, y, C=s, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=0, vmax=1)
    ax[1][0].axis([x.min(), x.max(), y.min(), y.max()])

    ax[1][1].set_title('Acquisition Function', fontdict={'size':15})
    im11 = ax[1][1].hexbin(x, y, C=ut, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=0, vmax=8)
    print(ut)
    maxVal_x = np.where(ut.reshape((300, 300)) == ut.max())[0]
    maxVal_y = np.where(ut.reshape((300, 300)) == ut.max())[1]
    print(np.where(ut.reshape((300, 300)) == ut.max()))
    print(maxVal_x)
    print(maxVal_y)

    ax[1][1].plot([np.where(ut.reshape((300, 300)) == ut.max())[1]/50.,
                   np.where(ut.reshape((300, 300)) == ut.max())[1]/50.],
                  [0, 6],
                  '-', lw=2, color='k')
    # plt.show()

    ax[1][1].plot([0, 6],
                  [np.where(ut.reshape((300, 300)) == ut.max())[0]/50.,
                   np.where(ut.reshape((300, 300)) == ut.max())[0]/50.],
                  '-', lw=2, color='k')
    # plt.show()

    ax[1][1].axis([x.min(), x.max(), y.min(), y.max()])

    for im, axis in zip([im00, im10, im01, im11], ax.flatten()):
        cb = fig.colorbar(im, ax=axis)
        # cb.set_label('Value')

    if name is None:
        name = '_'

    plt.tight_layout()
    plt.axis('equal')

    # Save or show figure?
    # fig.savefig('./figures/fourLine/'+'boa_eg_' + name + '.png')
    plt.show()
    plt.close(fig)

### target
def target(x, y):
    a = np.exp(x+y-7)
    b = np.exp(-x-y+5)
    c = np.exp(-x+y-2)
    d = np.exp(x-y-1)

    return 4-(a+b+c+d)

### inObj
def inObj(ptx,pty):
    seg_a = lambda x,y : x+y-7
    seg_b = lambda x,y : -x-y+5
    seg_c = lambda x,y : -x+y-2
    seg_d = lambda x,y : x-y-1
    if np.sign(seg_a(ptx,pty)) < 0 and np.sign(seg_b(ptx,pty)) < 0 and np.sign(seg_c(ptx,pty)) < 0 and np.sign(seg_d(ptx,pty)) < 0:
        return True
    else:
        return False

### gen_slide_path2
def gen_slide_path2(endPt, startPt={'x':0,'y':0}, d_c = 0.1):
    ex = list(endPt.values())[0]
    ey = list(endPt.values())[1]
    sx = list(startPt.values())[0]
    sy = list(startPt.values())[1]
    path_len = np.sqrt((ex-sx)**2+(ey-sy)**2)
    num_pt = int((path_len//d_c)+ 1)
    intptx = []
    intpty = []
    for i in range(num_pt-1):
        s = i / (num_pt - 1)
        curPtx = sx + s*(ex - sx)
        curPty = sy + s*(ey - sy)
        if not inObj(curPtx,curPty):
            intptx.append(curPtx)
            intpty.append(curPty)
        else:
            break
    return intptx, intpty

### set the distribution
n = 1e5
x = y = np.linspace(0, 6, 300)
X, Y = np.meshgrid(x, y)
Z = target(X, Y)
x = X.ravel()
y = Y.ravel()
XY = np.vstack([x, y]).T
z = target(x, y)

zmin = -5
zmax = 4
print(min(z))
print(max(z))
fig, axis = plt.subplots(1, 1, figsize=(14, 10))
gridsize=150

im = axis.hexbin(x, y, C=z, gridsize=gridsize, cmap=cm.jet, bins=None, vmin=zmin, vmax=zmax)
axis.axis([x.min(), x.max(), y.min(), y.max()])

cb = fig.colorbar(im, )
cb.set_label('Value')
# plot the boundary of box
xbd = [3,4,2.5,1.5,3]
ybd = [2,3,4.5,3.5,2]
plt.plot(xbd,ybd,'k-', lw=2, color='k')
axis.axis('equal')
plt.show()

# -------------- slide --------------
bo = BayesianOptimization(target, {'x': (0, 6), 'y': (0, 6)})
plt.ioff()
util = UtilityFunction(kind="ei", 
                    kappa = 2, 
                    xi=0.5,
                    kappa_decay=1,
                    kappa_decay_delay=0)
curPt = {'x':0,'y':0}
for i in range(50):
    nextPt = bo.suggest(util) # dict type
    # generate the slide segment
    intptx, intpty = gen_slide_path2(endPt=nextPt, startPt=curPt)
    
    # probe at these points (excluding start pt)
    for i in range(len(intptx))[1:]:
        # form the point in dict. type
        probePt_dict = {'x':intptx[i],'y':intpty[i]}
        probePtz = target(**probePt_dict)
        bo.register(params=probePt_dict, target=probePtz)
    
    # probe goes to the nextPt
    # curPt = copy.deepcopy(nextPt)
    curPt = {'x':intptx[-1],'y':intpty[-1]}
    plot_2d("{:03}".format(len(bo._space.params)))
# ============== slide ==============

##=== BOA related codes END ===##

if __name__ == '__main__':
    rospy.init_node("test_move")
    moveit_commander.roscpp_initialize(sys.argv)
    # # ur_control.speedl_control([0, -0.1, 0, 0, 0, 0], 0.5, 2)

    ############# ur control #############
    # ur_control = MoveGroupPythonInteface(sim=True)  #simu
    ur_control = MoveGroupPythonInteface(sim=False)  #real
    rospy.loginfo('init ok.')

    ur_control.group.get_planning_frame()
    ur_control.group.get_end_effector_link()

    ur_control.remove_allobjects()
    # setup_scene(ur_control)

    # Config UR

    res = ur_control.set_speed_slider(0.3)
    rospy.loginfo("set_speed_slider: {}".format(res))
    rospy.sleep(1)

    res = ur_control.play_program()
    rospy.loginfo("play_program: {}".format(res))
    rospy.sleep(1)
    # obtain the current pose list
    rospy.loginfo('current pose[xyzxyzw]: \n{}'.format(ur_control.get_pose_list()))
    rospy.loginfo('current joint: \n{}'.format(ur_control.get_joint_pos_list()))

    # # set the initial pos
    initPtx = ur_control.group.get_current_pose().pose.position.x
    initPty = ur_control.group.get_current_pose().pose.position.y
    initPtz = ur_control.group.get_current_pose().pose.position.z # surface plane
    # [one option] 
    # initPtx = -0.4865369575882814
    # initPty = -0.08797877559242727
    # initPtz = 0.07731254732208744 # -0.011764603861335071
    # [one option] 
    initPtx = -0.5931848696000094
    initPty = -0.28895797651231064
    initPtz = 0.07731254732208744 

    # x positive/ x negative/ y positive/ y negative
    xp = 0.23
    xn = 0.45
    yp = 0.22
    yn = 0
    
    # limit of x,y (in world frame)
    # xmax = initPtx + yn
    # xmin = initPtx - yp
    # ymax = initPty + xp
    # ymin = initPty - xn
    xmax = initPtx + 1
    xmin = initPtx - 1
    ymax = initPty + 1
    ymin = initPty - 1
    lim = [xmin, xmax, ymin, ymax]
    # # set zero to the ft 300
    zero_ft_sensor()
    
    # cal. each target
    Lrang = 0.3
    N = 2
    # theta_s = 30
    # delta_theta = (90 - theta_s)/N
    theta_s = 150
    delta_theta = 30
    
    saftz = initPtz + 0.10 # +10cm
    # PENETRATION DEPTH
    PENE_DEPTH = 0.15 #(default: -0.03) # <<<<<<
    depthz = initPtz + PENE_DEPTH
    # SAFE FORCE
    SAFE_FORCE = 15.0  #(default: 15)  <<<<<<
    flargeFlag = 0
    # folder name
    fd_name = '20220624D5L30/data/' # <<<<<<
    isSaveForce = 0           # <<<<<<
    # velocity limits setting
    maxVelScale = 0.7
    normalVelScale = 0.3

    listener = listener()
    
    #go the initial position
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
    
    for j in range(11,14): # <<<<<<
        #region： #experiment of 3 strokes    
        for i in range(1,4): #<<<<<<
            # current angle (deg)
            # theta_cur = theta_s + i * delta_theta
            theta_cur = 90
            # # current goal point (in task frame)
            # x_e_tskf = Lrang*math.cos(np.pi*theta_cur/180)
            # y_e_tskf = Lrang*math.sin(np.pi*theta_cur/180)
            # current goal point (in world frame)

            # start
            x_s_wldf = initPtx + (i-1) * 0.1
            y_s_wldf = initPty

            # goal
            x_e_wldf = initPtx + (i-1) * 0.1
            y_e_wldf = initPty + Lrang     

            # lift up to the start point
            waypoints = []
            wpose = ur_control.group.get_current_pose().pose
            # lift up
            wpose.position.z = saftz
            # wpose.position.z += 0.10
            waypoints.append(copy.deepcopy(wpose))
            (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
            ur_control.group.execute(plan, wait=True)
            
            # move to the start point            
            wpose.position.x = x_s_wldf
            wpose.position.y = y_s_wldf
            waypoints.append(copy.deepcopy(wpose))
            # (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
            if i != 1:
                ur_control.set_speed_slider(maxVelScale)
            (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
            ur_control.group.execute(plan, wait=True)
            ur_control.set_speed_slider(normalVelScale)

            # check the coorrdinate limit
            if checkCoorLimit([x_e_wldf, y_e_wldf], lim):               

                # penetration
                waypoints = []
                wpose.position.z = depthz
                waypoints.append(copy.deepcopy(wpose))
                (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
                ur_control.group.execute(plan, wait=True)
                ur_control.group.stop()
                rospy.sleep(2)                

                # go to the goal
                waypoints = []
                wpose.position.x = x_e_wldf
                wpose.position.y = y_e_wldf
                waypoints.append(copy.deepcopy(wpose))
                # (plan, fraction) = ur_control.group.compute_cartesian_path(waypoints,0.01,0.0)
                (plan, fraction) = ur_control.go_cartesian_path(waypoints,execute=False)
                listener.clear_finish_flag()
                zero_ft_sensor()
                ur_control.group.execute(plan, wait=False)
                rospy.loginfo('clear_finish_flag')
                while not listener.read_finish_flag():
                    # rospy.loginfo('Cur X: {}'.format(ur_control.group.get_current_pose().pose.position.x))
                    # rospy.loginfo('Cur Y: {}'.format(ur_control.group.get_current_pose().pose.position.y))
                    # rospy.loginfo('Cur Z: {}'.format(ur_control.group.get_current_pose().pose.position.z))
                    if listener.get_force_val() is not None:
                        # cal. the distance to the obj.
                        ## cur pos.
                        curx = ur_control.group.get_current_pose().pose.position.x
                        cury = ur_control.group.get_current_pose().pose.position.y
                        dist = np.abs(cury - y_s_wldf)
                        # measure the force val/dir
                        f_val = listener.get_force_val()
                        f_dir = listener.get_force_dir()
                        rospy.loginfo('Force Val( N ): {}'.format(f_val))
                        # rospy.loginfo('Force Dir(deg): {}'.format(f_dir))
                        if f_val > SAFE_FORCE:
                            rospy.loginfo('==== Large Force Warning ==== \n')
                            # emergency stop!!!
                            flargeFlag = emergency_stop(saftz)   
                            break
                        if isSaveForce ==  1:
                            ## start to record the data from Ft300
                            with open('/home/zhangzeqing/Nutstore Files/Nutstore/{}/znv_exp{}.csv'.format(fd_name,3*(j-1)+i),'a',newline="\n")as f:
                                f_csv = csv.writer(f) # <<<<<<
                                f_csv.writerow([np.round(f_val,6), np.round(f_dir,6), np.round(dist,6)])
                            f.close()
                rospy.loginfo('Ref Dir(deg): {}'.format(theta_cur))
                rospy.loginfo('{}-th loop finished'.format(i))
                
                # ur_control.group.stop()
                # rospy.sleep(3)

            else:
                rospy.loginfo('out of the worksapce:\n{}'.format(x_e_wldf,y_e_wldf))
                ur_control.group.stop()
            if flargeFlag == 1:
                break
        if flargeFlag == 1:
            break
    
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
