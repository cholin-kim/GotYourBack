from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8

from panda_kinematics import pandaVar
from omni_msgs.msg import OmniButtonEvent

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

from omni.omniKinematics import omniKinematics
from panda_kinematics.pandaKinematics import pandaKinematics
from cartesian_cmd import CartesianCmd
from phidget.FootPed import FootPed




class TeleopShared_single:
    def __init__(self, ns='R'):
        if not rospy.get_node_uri():
            rospy.init_node('teleop_shared_cmd')

        self.omni_kin = omniKinematics()
        self.panda_kin = pandaKinematics()
        self.CIC = CartesianCmd(arm=ns)
        self.ped = FootPed()
        
        if ns == 'L':
            robot_name = '/panda1'
            omni_name = '/omni1'
        elif ns == 'R':
            robot_name = '/panda2'
            omni_name = '/omni2'

        self.panda_q_pos = np.array(rospy.wait_for_message(robot_name + '/franka_state_controller/joint_states', JointState).position)
        self.omni_q_pos = np.array(rospy.wait_for_message(omni_name + '/omni/joint_states', JointState).position)
        self.fr3_q_pos = np.array([0.17048346850224572, -1.293712737872761, 0.06968514096397062, -2.580994415207858, -0.0494678581914531, 2.063796008375581, 0.16949485500424583])

        rospy.Subscriber(robot_name + '/franka_state_controller/joint_states', JointState, queue_size=1, callback=self.panda_q_cb)
        rospy.Subscriber(omni_name +'/omni/joint_states', JointState, queue_size=1, callback=self.omni_q_cb)
        self.pedal_pub = rospy.Publisher('/footped', UInt8, queue_size=1)

        # Variablesz
        self.scale = 0.8
        self.dpos = 0
        self.scaling = 0

        self.init_flag = True
        self.joint_limit_alert = False
        self.compliance_params_L = np.array([600, 20, 400, 15, 50, 5, 1.0])
        self.compliance_flag = False

        self.align_flag = False
        self.align_threshold = 0.15
        self.contact_flag = False
        self.contact_threshold = 0.02
        self.err_sum = 0

    def panda_q_cb(self, msg: JointState):
        self.panda_q_pos = np.array(msg.position)

        if not self.contact_flag:
            self.err_sum = 0

        if self.init_flag:
            pass
        else:
            self.compliance_params_L = np.array(list(self.CIC.get_compliance_params().values()))
        joint_margin = np.abs((self.panda_q_pos - (pandaVar.q_min + pandaVar.q_max) / 2)) / (
                    (pandaVar.q_max - pandaVar.q_min) / 2)
        # print(joint_margin)
        if (joint_margin[-1] > 0.6) or (joint_margin > 0.7).any() or self.compliance_flag:
            # if not self.init_flag:
            self.compliance_params_L[[0, 4, 6]] = [100.0, 3.0, 0.0]  # 1/40
            self.compliance_params_L[[1, 3]] = [4, 0.2]                             # 1/5
            self.CIC.set_compliance_params(self.compliance_params_L)
            print("Warning! Decreasing Stiffnesses")
            print(
                f"Config set to {self.compliance_params_L[0]}, {self.compliance_params_L[4]}, {self.compliance_params_L[6]}")
            self.joint_limit_alert = True
        else:
            if self.joint_limit_alert:
                if self.contact_flag:
                    self.compliance_params_L[[0, 2, 4, 6]] = self.compliance_params_L[[0, 2, 4, 6]] * 0.97 + np.array(
                        [100, 100, 60, 1.0]) * 0.03
                elif self.align_flag:
                    self.compliance_params_L[[0, 2, 4, 6]] = self.compliance_params_L[[0, 2, 4, 6]] * 0.97 + np.array(
                        [600, 100, 60, 1.0]) * 0.03
                else:
                    self.compliance_params_L[[0, 2, 4, 6]] = self.compliance_params_L[[0, 2, 4, 6]] * 0.97 + np.array(
                        [600, 400, 50, 1.0]) * 0.03
                # self.compliance_params[[1, 3]] = self.compliance_params[[1, 3]] * 0.95 + np.array([20, 5]) * 0.05
                self.CIC.set_compliance_params(self.compliance_params_L)
                if np.sum(self.compliance_params_L[[0, 4, 6]] - np.array([600, 60, 1.0])) >= 0:
                    print(
                        f"Config set to {self.compliance_params_L[0]}, {self.compliance_params_L[4]}, {self.compliance_params_L[6]}")
                    self.joint_limit_alert = False
                    self.CIC.set_compliance_params(self.compliance_params_L)
                    print('Deactivateing joint_limit_alert')

            else:
                if len(self.compliance_params_L) != 0:
                    self.CIC.set_compliance_params(self.compliance_params_L)



    def omni_q_cb(self, msg: JointState):
        self.omni_q_pos = np.array(msg.position)
        if self.omni_q_pos[-1] <= pandaVar.q_min[-1] * 0.8:
            self.omni_q_pos[-1] = pandaVar.q_min[-1] * 0.8
        if self.omni_q_pos[-1] >= pandaVar.q_max[-1] * 0.8:
            self.omni_q_pos[-1] = pandaVar.q_max[-1] * 0.8

        if self.omni_q_pos[-3] <= pandaVar.q_min[-3] * 0.8:
            self.omni_q_pos[-3] = pandaVar.q_min[-3] * 0.8
        if self.omni_q_pos[-3] >= pandaVar.q_max[-3] * 0.8:
            self.omni_q_pos[-3] = pandaVar.q_max[-3] * 0.8


    def teleop(self):
        print('start')
        flag_1 = True
        flag_2 = True

        t = 0

        while not rospy.is_shutdown():
            msg = UInt8()
            msg.data = self.ped.get_pedal_state()[0]
            self.pedal_pub.publish(msg)

            if abs(self.CIC.get_cur_T()[2, -1] - 0.27851) <= self.align_threshold:
                self.align_flag = True
                if flag_1:
                    self.compliance_flag = True
                    flag_1 = False
                else:
                    self.compliance_flag = False

            else:
                self.align_flag = False
                flag_1 = True

            if abs(self.CIC.get_cur_T()[2, -1] - 0.27851) <= self.contact_threshold:
                self.contact_flag = True
            else:
                self.contact_flag = False


            if self.ped.get_pedal_state()[0] == 1:
                Trb_ree_des = self.update_robot(m=self.omni_q_pos, s=self.panda_q_pos, sc=self.fr3_q_pos)
                Trb_ree_shared = np.copy(Trb_ree_des)
                if self.align_flag:
                    Trb_ree_shared[:3, :3] = R.from_euler('ZYX', [90, 0, 180], degrees=True).as_matrix()

                T_diff = Trb_ree_shared @ np.linalg.inv(self.CIC.get_cur_T())
                pos_diff = np.linalg.norm(np.abs(T_diff[:3, -1]))
                ori_diff = np.linalg.norm(R.from_matrix(T_diff[:3, :3]).as_rotvec(degrees=True))

                if ori_diff > 20:
                    if flag_2:
                        self.compliance_flag = True
                        flag_2 = False
                    else:
                        self.compliance_flag = False
                else:
                    flag_2 = True

                if self.contact_flag:
                    T_ee2ee = self.contact_manipulation()
                    Trb_ree_shared = Trb_ree_shared @ T_ee2ee


                Trb_ree_cmd = np.copy(Trb_ree_shared)

                self.CIC.set_pose_cmd_direct(Trb_ree_cmd)
                self.init_flag = False

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
        # Tcb_rb[:3, :3] = R.from_euler('Z', -np.pi / 2).as_matrix()  ##############
        # Tcb_rb[:3, -1] = [0.525, 0.525, 0]
        Tcb_rb[:3, :3] = R.from_euler('Z', np.pi / 2).as_matrix()  ##############
        Tcb_rb[:3, -1] = [0.025*16, -0.025*23, 0]


        Tcb_mb = np.identity(4)
        Tcb_mb[:3, :3] = R.from_euler('Z', -np.pi / 2).as_matrix()

        Tcam_ree_curr = Tcam_cb @ Tcb_rb @ Trb_ree_curr
        Tmb_mee = Tcam_cb @ Tcb_mb @ Tmb_mee_curr

        if self.init_flag:
            self.dpos = Tcam_ree_curr[:3, -1] - self.scale * Tmb_mee[:3, -1]

        pos_nominal = self.scale * Tmb_mee[:3, -1] + self.dpos
        Tmb_mee[:3, -1] = pos_nominal

        Trb_ree_des = np.linalg.inv(Tcam_cb @ Tcb_rb) @ Tmb_mee

        return Trb_ree_des

    def contact_manipulation(self):
        F_z_des = 5
        kp_z = 0.0001
        ki_z = 1

        F_z_err = F_z_des - self.CIC.F_ext[2]
        T_ee2ee = np.eye(4)
        T_ee2ee[2, -1] = kp_z * F_z_err + ki_z * self.err_sum * 0.001
        # print(T_ee2ee[2, -1])
        # if abs(T_ee2ee[2, -1]) >= 0.01:
        #     T_ee2ee[2, -1] = np.sign(T_ee2ee[2, -1]) * 0.01

        self.err_sum += F_z_err

        return T_ee2ee



if __name__ == '__main__':
    ts = TeleopShared_single(ns='R')
    ## 'R' currently not working
    ts.teleop()
