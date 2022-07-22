import math
import numpy as np
import rospy
import robotiq_ft_sensor.srv

import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

gripper_length = 0.089
gripper_length_collision = gripper_length - 0.005

conveyer_board_offset_x = 1.07
conveyer_board_offset_y = 0.2


def add_conveyer(self, timeout=4):
    conveyer_length = 4
    conveyer_width = 0.486
    conveyer_z = 0.8

    box_size = [conveyer_length + 0.2, conveyer_width + 0.02, conveyer_z]
    quat = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "board"
    box_pose.pose.position.x = conveyer_board_offset_x - conveyer_length / 2
    box_pose.pose.position.y = conveyer_board_offset_y - conveyer_width / 2
    box_pose.pose.position.z = 0 - box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]
    box_name = "conveyer"

    self.scene.add_box(box_name, box_pose, size=tuple(box_size))
    ret_conveyer = self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)
    return ret_conveyer


def add_camerabox(self, timeout=4):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "board"

    box_name = "camera_box"

    box_size = (0.3, 0.1, 0.5)
    quat = quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))
    box_pose.pose.position.x = -0.15
    box_pose.pose.position.y = 0.20 + box_size[1] / 2
    box_pose.pose.position.z = box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]

    self.scene.add_box(box_name, box_pose, size=box_size)

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "board"
    return self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)

def add_gripper(self, timeout=2):
    # return True
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"

    box_name = "gripper"

    box_size = (0.15, 0.2, 0.06)
    quat = quaternion_from_euler(*np.deg2rad(np.array([0,0,0])))
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = box_size[2] / 2 + 0.03
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]

    # self.scene.add_cylinder(box_name, box_pose, height=box_size[2], radius=box_size[0])
    self.scene.add_box(box_name, box_pose, size=box_size)
    ret1 = self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)
    if ret1:
        eef_link = 'tool0'
        self.scene.attach_box(eef_link, box_name, touch_links=['wrist_3_link'])
        return self.wait_for_state_update(box_name, object_is_attached=True, object_is_known=False, timeout=timeout)


def add_attach_objectbox(self, timeout=2):
    # return True
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "tool0"

    box_name = "object_box"

    box_size = (0.15, 0.15, 0.01)
    # quat = quaternion_from_euler(*np.radians([0, 90, 90])) #obj2ee
    quat = quaternion_from_euler(*np.radians([0, 0, 0]))
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = gripper_length_collision + box_size[2] / 2
    box_pose.pose.orientation.x = quat[0]
    box_pose.pose.orientation.y = quat[1]
    box_pose.pose.orientation.z = quat[2]
    box_pose.pose.orientation.w = quat[3]

    self.scene.add_box(box_name, box_pose, size=box_size)

    ret1 = self.wait_for_state_update(box_name, object_is_known=True, timeout=timeout)
    if ret1:
        eef_link = 'tool0'
        self.scene.attach_box(eef_link, box_name, touch_links=[eef_link, 'wrist_3_link'])
        return self.wait_for_state_update(box_name, object_is_attached=True, object_is_known=False, timeout=timeout)


def remove_object_box(self, timeout=2):
    # return True
    eef_link = 'tool0'
    box_name = "object_box"
    self.scene.remove_attached_object(eef_link, name=box_name)
    ret1 = self.wait_for_state_update(box_name, object_is_known=True, object_is_attached=False, timeout=timeout)
    self.scene.remove_world_object(box_name)
    return self.wait_for_state_update(box_name, object_is_attached=False, object_is_known=False, timeout=timeout)


def setup_scene(self):
    # add_conveyer(self)
    # add_camerabox(self)
    add_gripper(self)

def zero_ft_sensor():
    srv_name = '/robotiq_ft_sensor_acc'
    rospy.wait_for_service(srv_name)
    try:
        srv_fun = rospy.ServiceProxy(srv_name, robotiq_ft_sensor.srv.sensor_accessor)
        resp1 = srv_fun(robotiq_ft_sensor.srv.sensor_accessorRequest.COMMAND_SET_ZERO, '')
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

import threading
from robotiq_ft_sensor.msg import ft_sensor
from control_msgs.msg import FollowJointTrajectoryActionResult as rlst
class ft_listener():
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
    

    # # rospy.loginfo(msg)
    
    def read_sensor(self):
        with self.lock_read:
            return self.sensor_data
    
    def get_force_val(self):
        with self.lock_read:
            return self.force_val

    def get_force_dir(self):
        with self.lock_read:
            return self.force_dir

## --- [force monitor] ---
# def force_monitor():
#     rospy.loginfo('clear_finish_flag')
#     while not listener.read_finish_flag():                    
#         ## measure the force val/dir
#         f_val = listener.get_force_val()
#         f_dir = listener.get_force_dir()
#         if f_val is not None:
#             ## most conservative way (most safe)
#             if np.round(f_val,6) > CUR_SAFE_FORCE:
#                 rospy.loginfo('==== Large Force Warning ==== \n')
#                 ur_control.group.stop()
#                 flargeFlag = True
#                 break

#             ## path distance
#             cur_pos = ur_control.group.get_current_pose().pose
#             curx = cur_pos.position.x
#             cury = cur_pos.position.y
#             dist = round(np.abs(cury - y_s_wldf),6)
            
#             # df_ls.append(round(f_val,6))
#             # dr_ls.append(round(f_dir,6))
#             # ds_ls.append(dist)
#             # rospy.loginfo('ForceVal (N): {}'.format(f_val))
#             # rospy.loginfo('Distance (m): {}'.format(dist))
            
#             ### only record the stable duration
#             if round(dist - ds_min, 6) >= 0:
#                 df_ls.append(round(f_val,6))
#                 dr_ls.append(round(f_dir,6))
#                 ds_ls.append(dist)
            
#             ## using jamming detector 1
#             if len(df_ls) >= ite_bar and len(df_ls)%delta_ite==0:
#                 ## Kalman Filter
#                 fdhat, Pminus = smooth_fd_kf(df_ls)
#                 ## Mean
#                 fdmean = get_mean(df_ls)
#                 ## absolute difference (N)
#                 cur_abs_diff = round(abs(fdhat[-1] - fdmean[-1]),3)
#                 ## detect jamming
#                 # Mx, isJamming = jd1(fdhat, fdmean, 0.5, delta_ite) 
#                 JDlib = JDLib(df_ls,ds_ls,fdhat,fdmean,diff_bar)
#                 isJamming = JDlib.JD(JDid)
#                 if isJamming:
#                     rospy.loginfo('**== Jamming Detected ==** \n')
#                     ## ----- v1.0 -----
#                     ur_control.group.stop()
#                     flargeFlag = True
#                     ## plot jamming result
#                     if isPlotJD:
#                         ds_adv = round(ds_obj-ds_ls[-1], 3) # >0 in theory
#                         title_str = 'Exp{}: ds [{},{}], Dep {}, Vel {}, Ite {}, Diff {}, Adv {}'.format(j,ds_min,np.inf,PENE_DEPTH,normalVelScale,len(df_ls),cur_abs_diff,ds_adv)
#                         JDlib.plotJDRes(ds_obj,title_str,fig_path,j)
#                     break
    