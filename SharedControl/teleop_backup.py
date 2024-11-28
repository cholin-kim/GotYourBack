from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from omni_msgs.msg import OmniButtonEvent
# from geometry_msgs.msg import Vector3

from omni.omniKinematics import omniKinematics
from panda_kinematics.pandaKinematics import pandaKinematics
import rospy

import numpy as np
from scipy.spatial.transform import Rotation as R
from cartesian_cmd import CartesianCmd
from SharedControl.phidget.FootPed import FootPed
# from DHgripper.DHGripperROS import DHGripperROS


class TeleopShared:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('teleop_shared_cmd')
        self.panda_q_pos = np.array(rospy.wait_for_message('/panda1/franka_state_controller/joint_states', JointState).position)
        self.omni_q_pos = np.array(rospy.wait_for_message('/omni1/joint_states', JointState).position)
        # self.fr3_q_pos = np.array([.10694790665028135, -1.7430189460193444, -0.049731157788515674, -2.302841384502822, -0.21950222399128386, 2.1893147779269446, -0.005798781875155287]) # currently fixed
        self.fr3_q_pos = np.array([1.723878620949372, -1.734472929133216, 0.0939505587795933, -2.298456468235122, -0.21973442834581253, 2.2121563758885388, 0.1269505113718299])
        
        rospy.Subscriber('/panda1/franka_state_controller/joint_states', JointState, queue_size=1, callback=self.panda_q_cb)
        rospy.Subscriber('/omni1/joint_states', JointState, queue_size=1, callback=self.omni_q_cb)
        # self.pub_omni_feedback = rospy.Publisher('/omni1/force_feedback', OmniFeedback, queue_size=1)

        self.omni_kin = omniKinematics()
        self.panda_kin = pandaKinematics()
        self.CIC = CartesianCmd()
        self.ped = FootPed()
        # self.gripper = DHGripperROS()

        # Variables
        self.scale= 1.0

    
    def panda_q_cb(self, msg:JointState):
        self.panda_q_pos = np.array(msg.position)

    def omni_q_cb(self, msg:JointState):
        self.omni_q_pos = np.array(msg.position)
    
    def teleop(self):
        T_mb2sb = np.eye(4)
        T_mb2sb[:3, :3] = R.from_euler('z', -np.pi/2).as_matrix()
    
        T_me2se = np.eye(4)
        T_me2se[:3, :3] = R.from_euler('YX', [np.pi/2, np.pi]).as_matrix()
    
        T_mb2me = self.omni_kin.fk(self.omni_q_pos)[0][-1]
        T_sb2se = self.panda_kin.fk(self.panda_q_pos)[0][-1]
    
        T_sb2se_ = np.linalg.inv(T_mb2sb) @ T_mb2me @ T_me2se
    
        ee_offset = T_sb2se[:3, -1] - T_sb2se_[:3, -1]
        print(ee_offset)
    
    
        T_sb2se_cur = np.copy(T_sb2se_)
        T_sb2se_[:3, -1] += ee_offset
    
        ee_pos_cmd = np.zeros(3)
        ee_rot_cmd = np.eye(3)
    
        print('start')
        init_flag = True
        while not rospy.is_shutdown():
            T_mb2me = self.omni_kin.fk(self.omni_q_pos)[0][-1]
            T_sb2se = self.panda_kin.fk(self.panda_q_pos)[0][-1]
            T_sb2se_new = np.linalg.inv(T_mb2sb) @ T_mb2me @ T_me2se
    
    
            ee_rot_cmd = np.linalg.inv(T_sb2se_cur[:3, :3]) @ T_sb2se_new[:3 , :3]
            T_sb2se_[:3, :3] = T_sb2se_[:3, :3] @ ee_rot_cmd
    
    
            T_sb2se_m = np.copy(T_sb2se)
            T_sb2se_m[:3, -1] -= ee_offset
            T_mb2me_ = T_mb2sb @ T_sb2se_m @ np.linalg.inv(T_me2se)
    
            if self.ped.get_pedal_state()[0] == 1:
                ee_pos_cmd = T_sb2se_new[:3, -1] - T_sb2se_cur[:3, -1]
                T_sb2se_[:3, -1] += ee_pos_cmd * 1.5

                if init_flag:
                    self.CIC.set_pose_cmd(T_sb2se_, duration=3)
                    init_flag = False
                else:
                    self.CIC.set_pose_cmd_direct(T_sb2se_)
                # self.omni_feedback(None)
                # self.omni_feedback(T_mb2me_[:3, -1])
    
            elif self.ped.get_pedal_state()[0] == 0:
                init_flag = True
            #                 self.omni_feedback(None)
    
            T_sb2se_cur = np.copy(T_sb2se_new)
            # self.gripper.gripper_cmd()
            rospy.sleep(0.001)


if __name__=='__main__':
    ts = TeleopShared()
    ts.teleop()





