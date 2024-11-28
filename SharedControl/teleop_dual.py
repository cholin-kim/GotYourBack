from sensor_msgs.msg import JointState
from panda_kinematics import pandaVar
from std_msgs.msg import UInt8

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

class TeleopShared_dual:
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('teleop_shared_cmd')

        self.omni_kin = omniKinematics()
        self.panda_kin = pandaKinematics()
        self.CIC_L = CartesianCmd(arm='L')
        self.CIC_R = CartesianCmd(arm='R')
        self.ped = FootPed()
        # self.gripper = DHGripperROS()


        self.panda1_q_pos = np.array(
            rospy.wait_for_message('/panda1/franka_state_controller/joint_states', JointState).position)
        self.panda2_q_pos = np.array(
            rospy.wait_for_message('/panda2/franka_state_controller/joint_states', JointState).position)
        self.omni_1_q_pos = np.array(rospy.wait_for_message('/omni1/joint_states', JointState).position)
        self.omni_2_q_pos = np.array(rospy.wait_for_message('/omni2/joint_states', JointState).position)
        self.fr3_q_pos = np.array(
            [0.3230508194141213, -1.1348287612256966, 0.21110954878309973, -2.1477468616621564, -0.20662960747218065, 1.6709500585526351, 0.458321223884512])

        rospy.Subscriber('/panda1/franka_state_controller/joint_states', JointState, queue_size=1,
                         callback=self.panda_1_q_cb)
        rospy.Subscriber('/panda2/franka_state_controller/joint_states', JointState, queue_size=1,
                         callback=self.panda_2_q_cb)
        rospy.Subscriber('/omni1/joint_states', JointState, queue_size=1, callback=self.omni_1_q_cb)
        rospy.Subscriber('/omni2/joint_states', JointState, queue_size=1, callback=self.omni_2_q_cb)
        # self.pub_omni_feedback = rospy.Publisher('/omni1/force_feedback', OmniFeedback, queue_size=1)
        self.pedal_pub = rospy.Publisher('/footped', UInt8, queue_size=1)

        # Variables
        self.scale= 1.0
        self.dpos_L = 0
        self.dpos_R = 0
        self.scaling = 0

        self.init_flag = True
        self.init_flag2 = True
        self.joint_limit_alert = False
        self.compliance_params_L = np.array([600, 20, 400, 20, 50, 5, 0.0])
        self.compliance_params_R = np.array([600, 20, 400, 20, 50, 5, 0.0])


    
    def panda_1_q_cb(self, msg:JointState):
        self.compliance_params_L = np.array(list(self.CIC_L.get_compliance_params().values()))
        self.panda1_q_pos = np.array(msg.position)
        joint_margin = np.abs((self.panda1_q_pos - (pandaVar.q_min + pandaVar.q_max) / 2)) / ((pandaVar.q_max - pandaVar.q_min) / 2)
        # print(joint_margin)
        if (joint_margin[-1] > 0.8) or (joint_margin > 0.9).any():
            if not self.init_flag:
                self.compliance_params_L[[0, 4, 6]] = [100.0, 3.0, 0.0]               # 1/40
                # self.compliance_params[[1, 3]] = [4, 0.2]                             # 1/5
                self.CIC_L.set_compliance_params(self.compliance_params_L)
                print("Warning! Approaching Joint Limit!!")
                print(f"Config set to {self.compliance_params_L[0]}, {self.compliance_params_L[1]}, {self.compliance_params_L[2]}, {self.compliance_params_L[3]}, {self.compliance_params_L[4]}")
                self.joint_limit_alert = True
        else:
            if self.joint_limit_alert:
                self.compliance_params_L[[0, 4, 6]] = self.compliance_params_L[[0, 4, 6]] * 0.97 + np.array([400, 50, 0.0]) * 0.03
                # self.compliance_params[[1, 3]] = self.compliance_params[[1, 3]] * 0.95 + np.array([20, 5]) * 0.05
                self.CIC_L.set_compliance_params(self.compliance_params_L)
                if np.sum(self.compliance_params_L[[0, 4, 6]] - np.array([400, 50, 0.0])) >= 0:
                    print(
                        f"Config set to {self.compliance_params_L[0]}, {self.compliance_params_L[1]}, {self.compliance_params_L[2]}, {self.compliance_params_L[3]}, {self.compliance_params_L[4]}")
                    self.joint_limit_alert = False
                    self.CIC_L.set_compliance_params(self.compliance_params_L)
                    print('Deactivateing joint_limit_alert')

            else:
                if len(self.compliance_params_L) != 0:
                    self.CIC_L.set_compliance_params(self.compliance_params_L)

    def panda_2_q_cb(self, msg:JointState):
        self.compliance_params_R = np.array(list(self.CIC_R.get_compliance_params().values()))
        self.panda2_q_pos = np.array(msg.position)
        joint_margin = np.abs((self.panda2_q_pos - (pandaVar.q_min + pandaVar.q_max) / 2)) / ((pandaVar.q_max - pandaVar.q_min) / 2)
        # print(joint_margin)
        if (joint_margin[-1] > 0.8) or (joint_margin > 0.9).any():
            if not self.init_flag:
                self.compliance_params_R[[0, 4, 6]] = [100.0, 3.0, 0.0]               # 1/40
                # self.compliance_params[[1, 3]] = [4, 0.2]                             # 1/5
                self.CIC_R.set_compliance_params(self.compliance_params_R)
                print("Warning! Approaching Joint Limit!!")
                print(f"Config set to {self.compliance_params_R[0]}, {self.compliance_params_R[1]}, {self.compliance_params_R[2]}, {self.compliance_params_R[3]}, {self.compliance_params_R[4]}")
                self.joint_limit_alert = True
        else:
            if self.joint_limit_alert:
                self.compliance_params_R[[0, 4, 6]] = self.compliance_params_R[[0, 4, 6]] * 0.97 + np.array([400, 50, 0.0]) * 0.03
                # self.compliance_params[[1, 3]] = self.compliance_params[[1, 3]] * 0.95 + np.array([20, 5]) * 0.05
                self.CIC_R.set_compliance_params(self.compliance_params_R)
                if np.sum(self.compliance_params_R[[0, 4, 6]] - np.array([400, 50, 0.0])) >= 0:
                    print(
                        f"Config set to {self.compliance_params_R[0]}, {self.compliance_params_R[1]}, {self.compliance_params_R[2]}, {self.compliance_params_R[3]}, {self.compliance_params_R[4]}")
                    self.joint_limit_alert = False
                    self.CIC_R.set_compliance_params(self.compliance_params_R)
                    print('Deactivateing joint_limit_alert')

            else:
                if len(self.compliance_params_R) != 0:
                    self.CIC_R.set_compliance_params(self.compliance_params_R)

    def omni_1_q_cb(self, msg:JointState):
        self.omni_1_q_pos = np.array(msg.position)
        if self.omni_1_q_pos[-1] <= pandaVar.q_min[-1] * 0.8:
            self.omni_1_q_pos[-1] = pandaVar.q_min[-1] * 0.8
        if self.omni_1_q_pos[-1] >= pandaVar.q_max[-1] * 0.8:
            self.omni_1_q_pos[-1] = pandaVar.q_max[-1] * 0.8

        if self.omni_1_q_pos[-3] <= pandaVar.q_min[-3] * 0.8:
            self.omni_1_q_pos[-3] = pandaVar.q_min[-3] * 0.8
        if self.omni_1_q_pos[-3] >= pandaVar.q_max[-3] * 0.8:
            self.omni_1_q_pos[-3] = pandaVar.q_max[-3] * 0.8

    def omni_2_q_cb(self, msg:JointState):
        self.omni_2_q_pos = np.array(msg.position)
        if self.omni_2_q_pos[-1] <= pandaVar.q_min[-1] * 0.8:
            self.omni_2_q_pos[-1] = pandaVar.q_min[-1] * 0.8
        if self.omni_2_q_pos[-1] >= pandaVar.q_max[-1] * 0.8:
            self.omni_2_q_pos[-1] = pandaVar.q_max[-1] * 0.8

        if self.omni_2_q_pos[-3] <= pandaVar.q_min[-3] * 0.8:
            self.omni_2_q_pos[-3] = pandaVar.q_min[-3] * 0.8
        if self.omni_2_q_pos[-3] >= pandaVar.q_max[-3] * 0.8:
            self.omni_2_q_pos[-3] = pandaVar.q_max[-3] * 0.8

    def teleop(self):
        print('start')

        while not rospy.is_shutdown():
            msg = UInt8()
            msg.data = self.ped.get_pedal_state()[0]
            self.pedal_pub.publish(msg)

            if self.ped.get_pedal_state()[0] == 1:
                L_Trb_ree_des = self.update_robot_L(m=self.omni_1_q_pos, s=self.panda1_q_pos, sc=self.fr3_q_pos)
                R_Trb_ree_des = self.update_robot_R(m=self.omni_2_q_pos, s=self.panda2_q_pos, sc=self.fr3_q_pos)
                if self.init_flag:
                    du = 1
                    L_Ts = self.CIC_L.interpolate_pose(L_Trb_ree_des, duration=du)
                    R_Ts = self.CIC_R.interpolate_pose(R_Trb_ree_des, duration=du)
                    self.init_flag = False
                    for i in range(len(L_Ts)):
                        self.CIC_L.set_pose_cmd_direct(L_Ts[i])
                        self.CIC_R.set_pose_cmd_direct(R_Ts[i])
                        rospy.sleep(0.01)

                else:
                    self.CIC_L.set_pose_cmd_direct(L_Trb_ree_des)
                    self.CIC_R.set_pose_cmd_direct(R_Trb_ree_des)
                    # print(1)
                    pass

            elif self.ped.get_pedal_state()[0] == 0:
                self.init_flag = True

    def update_robot_L(self, m, s, sc):
        # Goal : Tmb_mee = Tcam_ree
        Tmb_mee_curr = self.omni_kin.fk(m)[0][-1]
        Tmb_mee_curr[:3, :3] = Tmb_mee_curr[:3, :3] @ R.from_euler('Z', np.pi/2).as_matrix()
        Trb_ree_curr = self.panda_kin.fk(s)[0][-1]

        Tcb_ee = self.panda_kin.fk_cam(sc)[0][-1]
        Tee_cam = np.identity(4)
        Tee_cam[2, -1] = 0.14
        Tcb_cam = Tcb_ee @ Tee_cam
        Tcam_cb = np.linalg.inv(Tcb_cam)

        Tcb_rb = np.identity(4)
        Tcb_rb[:3, :3] = R.from_euler('Z', -np.pi/2).as_matrix()
        Tcb_mb = np.identity(4)
        Tcb_mb[:3, :3] = R.from_euler('Z', -np.pi/2).as_matrix()

        Tcam_ree_curr = Tcam_cb @ Tcb_rb @ Trb_ree_curr
        Tmb_mee = Tcam_cb @ Tcb_mb @ Tmb_mee_curr

        if self.init_flag:
            self.dpos_L = Tcam_ree_curr[:3, -1] - self.scale * Tmb_mee[:3, -1]

        pos_nominal = self.scale * Tmb_mee[:3, -1] + self.dpos_L
        Tmb_mee[:3, -1] = pos_nominal

        Trb_ree_des = np.linalg.inv(Tcam_cb @ Tcb_rb) @ Tmb_mee


        return Trb_ree_des

    def update_robot_R(self, m, s, sc):
        # Goal : Tmb_mee = Tcam_ree
        Tmb_mee_curr = self.omni_kin.fk(m)[0][-1]
        Tmb_mee_curr[:3, :3] = Tmb_mee_curr[:3, :3] @ R.from_euler('Z', -np.pi / 2).as_matrix()
        Trb_ree_curr = self.panda_kin.fk(s)[0][-1]

        Tcb_ee = self.panda_kin.fk_cam(sc)[0][-1]
        Tee_cam = np.identity(4)
        Tee_cam[2, -1] = 0.14
        Tcb_cam = Tcb_ee @ Tee_cam
        Tcam_cb = np.linalg.inv(Tcb_cam)

        Tcb_rb = np.identity(4)
        Tcb_rb[:3, :3] = R.from_euler('Z', np.pi / 2).as_matrix()
        Tcb_mb = np.identity(4)
        Tcb_mb[:3, :3] = R.from_euler('Z', -np.pi/2).as_matrix()

        Tcam_ree_curr = Tcam_cb @ Tcb_rb @ Trb_ree_curr
        Tmb_mee = Tcam_cb @ Tcb_mb @ Tmb_mee_curr

        if self.init_flag:
            self.dpos_R = Tcam_ree_curr[:3, -1] - self.scale * Tmb_mee[:3, -1]

        pos_nominal = self.scale * Tmb_mee[:3, -1] + self.dpos_R
        Tmb_mee[:3, -1] = pos_nominal

        Trb_ree_des = np.linalg.inv(Tcam_cb @ Tcb_rb) @ Tmb_mee


        return Trb_ree_des


if __name__=='__main__':
    ts = TeleopShared_dual()
    ts.teleop()





