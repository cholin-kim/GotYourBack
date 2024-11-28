from sensor_msgs.msg import JointState
# from omni_msgs.msg import OmniFeedback
# from omni_msgs.msg import OmniButtonEvent
# from geometry_msgs.msg import Vector3
from panda_kinematics import pandaVar

# import os
# filepath = os.path.abspath(__file__)
# split = filepath.split('/')
# path = '/home/'+ split[2] + '/' + split[3] + '/' + split[4]
# os.path.join(path, "")
# import sys
# sys.path.append("/home/surglab/GotYourBack")
# sys.path.append("/home/surglab/GotYourBack/Basler")

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R

from omni.omniKinematics import omniKinematics
from panda_kinematics.pandaKinematics import pandaKinematics
from cartesian_cmd import CartesianCmd
from phidget.FootPed import FootPed
from SharedControl.phidget.FootPed import FootPed
# from DHgripper.DHGripperROS import DHGripperROS
# from Basler.image_streaming import Streaming
# stream = Streaming()

class TeleopShared:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('teleop_shared_cmd')

        self.omni_kin = omniKinematics()
        self.panda_kin = pandaKinematics()
        self.CIC = CartesianCmd()
        self.ped = FootPed()
        # self.gripper = DHGripperROS()


        self.panda_q_pos = np.array(
            rospy.wait_for_message('/panda1/franka_state_controller/joint_states', JointState).position)
        self.omni_q_pos = np.array(rospy.wait_for_message('/omni1/joint_states', JointState).position)
        # self.fr3_q_pos = np.array(
        #     [0.6783777698255324, -1.5268527289371776, 0.3255198575292201, -2.3628464174158683, -0.2989773703440436,
        #      1.9794206613813092, 0.5670366882356068])
        self.fr3_q_pos = np.array(
            [0.17525871475099883, -1.3327882137310494, 0.4029727365051227, -1.658343168410214, -0.14972178010376444,
             1.3351961360523246, 0.469825315318599])


        rospy.Subscriber('/panda1/franka_state_controller/joint_states', JointState, queue_size=1,
                         callback=self.panda_q_cb)
        rospy.Subscriber('/omni1/joint_states', JointState, queue_size=1, callback=self.omni_q_cb)
        # self.pub_omni_feedback = rospy.Publisher('/omni1/force_feedback', OmniFeedback, queue_size=1)

        # Variables
        self.scale= 0.8
        self.dpos = 0
        self.scaling = 0

        self.init_flag = True
        self.joint_limit_alert = False
        self.compliance_params = np.array([400, 20, 50, 5, 0.0])


    
    def panda_q_cb(self, msg:JointState):
        self.panda_q_pos = np.array(msg.position)
        joint_margin = np.abs((self.panda_q_pos - (pandaVar.q_min + pandaVar.q_max) / 2)) / ((pandaVar.q_max - pandaVar.q_min) / 2)
        print(joint_margin)
        if (joint_margin[-1] > 0.4) or (joint_margin > 0.5).any():
            if not self.init_flag:
                self.compliance_params[[0, 2, 4]] = [100.0, 3.0, 0.0]               # 1/40
                # self.compliance_params[[1, 3]] = [4, 0.2]                             # 1/5
                self.CIC.set_compliance_params(self.compliance_params)
                print("Warning! Approaching Joint Limit!!")
                print(f"Config set to {self.compliance_params[0]}, {self.compliance_params[1]}, {self.compliance_params[2]}, {self.compliance_params[3]}, {self.compliance_params[4]}")
                self.joint_limit_alert = True
        else:
            if self.joint_limit_alert:
                self.compliance_params[[0, 2, 4]] = self.compliance_params[[0, 2, 4]] * 0.97 + np.array([400, 50, 0.0]) * 0.03
                # self.compliance_params[[1, 3]] = self.compliance_params[[1, 3]] * 0.95 + np.array([20, 5]) * 0.05
                self.CIC.set_compliance_params(self.compliance_params)
                if np.sum(self.compliance_params[[0, 2, 4]] - np.array([400, 50, 0.0])) >= 0:
                    print(
                        f"Config set to {self.compliance_params[0]}, {self.compliance_params[1]}, {self.compliance_params[2]}, {self.compliance_params[3]}, {self.compliance_params[4]}")
                    self.joint_limit_alert = False
                    self.CIC.set_compliance_params(self.compliance_params)
                    print('Deactivateing joint_limit_alert')

            else:
                self.CIC.set_compliance_params(self.compliance_params)

    def omni_q_cb(self, msg:JointState):
        self.omni_q_pos = np.array(msg.position)
        if self.omni_q_pos[-1] <= pandaVar.q_min[-1] * 0.7:
            self.omni_q_pos[-1] = pandaVar.q_min[-1] * 0.7
        if self.omni_q_pos[-1] >= pandaVar.q_max[-1] * 0.7:
            self.omni_q_pos[-1] = pandaVar.q_max[-1] * 0.7

        if self.omni_q_pos[-3] <= pandaVar.q_min[-3] * 0.7:
            self.omni_q_pos[-3] = pandaVar.q_min[-3] * 0.7
        if self.omni_q_pos[-3] >= pandaVar.q_max[-3] * 0.7:
            self.omni_q_pos[-3] = pandaVar.q_max[-3] * 0.7



    
    def teleop(self):
        print('start')

        while not rospy.is_shutdown():
            if self.ped.get_pedal_state()[0] == 1:
                Trb_ree_des = self.update_robot(m=self.omni_q_pos, s=self.panda_q_pos, sc=self.fr3_q_pos)
                if self.init_flag:
                    self.CIC.set_pose_cmd(Trb_ree_des, duration=3)
                    self.init_flag = False
                else:
                    self.CIC.set_pose_cmd_direct(Trb_ree_des)
                    # print(1)
                    pass

            elif self.ped.get_pedal_state()[0] == 0:
                self.init_flag = True




    def update_robot(self, m, s, sc):
        # Goal : Tmb_mee = Tcam_ree
        Tmb_mee_curr = self.omni_kin.fk(m)[0][-1]
        Trb_ree_curr = self.panda_kin.fk(s)[0][-1]

        Tcb_ee = self.panda_kin.fk(sc)[0][-1]
        Tee_cam = np.identity(4)
        Tee_cam[2, -1] = 0.14
        Tcb_cam = Tcb_ee @ Tee_cam
        Tcam_cb = np.linalg.inv(Tcb_cam)

        Tcb_rb = np.identity(4)
        Tcb_mb = np.identity(4)
        Tcb_mb[:3, :3] = R.from_euler('Z', -np.pi/2).as_matrix()

        Tcam_ree_curr = Tcam_cb @ Tcb_rb @ Trb_ree_curr
        Tmb_mee = Tcam_cb @ Tcb_mb @ Tmb_mee_curr

        if self.init_flag:
            self.dpos = Tcam_ree_curr[:3, -1] - self.scale * Tmb_mee[:3, -1]

        pos_nominal = self.scale * Tmb_mee[:3, -1] + self.dpos
        Tmb_mee[:3, -1] = pos_nominal

        Trb_ree_des = np.linalg.inv(Tcam_cb @ Tcb_rb) @ Tmb_mee


        return Trb_ree_des


if __name__=='__main__':
    ts = TeleopShared()
    ts.teleop()





