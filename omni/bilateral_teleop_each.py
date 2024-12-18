import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omniKinematics import omniKinematics
from omni_msgs.msg import OmniFeedback
import matplotlib.pyplot as plt


class touch_syn():
    def __init__(self):
        rospy.init_node('omni_each_PID')
        rospy.Subscriber('/ns1/omni1/joint_states', JointState, self.omni_cb_1)
        rospy.Subscriber('/ns2/omni1/joint_states', JointState, self.omni_cb_2)

        self.cur_pose_1 = np.array(rospy.wait_for_message('/ns1/omni1/joint_states', JointState, timeout=5).position)
        self.cur_pose_2 = np.array(rospy.wait_for_message('/ns2/omni1/joint_states', JointState, timeout=5).position)

        print('hi')
        self.pub_1 = rospy.Publisher('/ns1/omni_R/force_feedback', OmniFeedback, queue_size=1)
        self.pub_2 = rospy.Publisher('/ns2/omni_L/force_feedback', OmniFeedback, queue_size=1)
        self.feedback_1 = OmniFeedback()
        self.feedback_2 = OmniFeedback()

        self.kin = omniKinematics()
        a, _ = self.kin.fk(self.cur_pose_1)
        b, _ = self.kin.fk(self.cur_pose_2)
        self.prev_position_1 = a[-1][:3, 3]
        self.prev_position_2 = b[-1][:3, 3]

        # /--- 추가됨 #
        self.e_i_1 = np.zeros(3)
        self.e_p_1 = np.zeros(3)
        self.e_i_2 = np.zeros(3)
        self.e_p_2 = np.zeros(3)
        self.time_slice = 1 # 알아내야 하는 값
        #  --- ---/ #

    def omni_cb_1(self, msg:JointState):
        self.cur_pose_1 = np.array(msg.position)

    def omni_cb_2(self, msg:JointState):
        self.cur_pose_2 = np.array(msg.position)


    def PID(self):
        # parameters
        x_K_p = 70
        x_K_i = 0
        x_K_d = 10

        y_K_p = 70
        y_K_i = 0
        y_K_d = 10

        z_K_p = 70
        z_K_i = 0
        z_K_d = 10

        # Motor 힘에 따른 parameters
        # constant_x = 2
        # constant_y = 5
        # constant_z = 15

        rospy.Rate(1000)

        while not rospy.is_shutdown():
            a, _ = self.kin.fk(self.cur_pose_1)
            b, _ = self.kin.fk(self.cur_pose_2)

            current_1 = a[-1][:3, 3]
            current_2 = b[-1][:3, 3]

            desired_1 = b[-1][:3, 3]
            desired_2 = a[-1][:3, 3]


            e_p_1 = desired_1 - current_1
            self.e_i_1 += e_p_1
            e_d_1 = (e_p_1 - self.e_p_1)
            self.e_p_1 = e_p_1

            e_p_2 = desired_2 - current_2
            self.e_i_2 += e_p_2
            e_d_2 = (e_p_2 - self.e_p_2)
            self.e_p_2 = e_p_2

            control_x_1 = (x_K_p * e_p_1[0] + x_K_i * self.e_i_1[0] + x_K_d * e_d_1[0])
            control_y_1 = (y_K_p * e_p_1[1] + y_K_i * self.e_i_1[1] + y_K_d * e_d_1[1])
            control_z_1 = (z_K_p * e_p_1[2] + z_K_i * self.e_i_1[2] + z_K_d * e_d_1[2])

            control_x_2 = (x_K_p * e_p_2[0] + x_K_i * self.e_i_2[0] + x_K_d * e_d_2[0])
            control_y_2 = (y_K_p * e_p_2[1] + y_K_i * self.e_i_2[1] + y_K_d * e_d_2[1])
            control_z_2 = (z_K_p * e_p_2[2] + z_K_i * self.e_i_2[2] + z_K_d * e_d_2[2])

            self.feedback_1.force.x = control_x_1
            self.feedback_1.force.y = control_y_1
            self.feedback_1.force.z = control_z_1

            self.feedback_2.force.x = control_x_2
            self.feedback_2.force.y = control_y_2
            self.feedback_2.force.z = control_z_2

            self.pub_1.publish(self.feedback_1)
            self.pub_2.publish(self.feedback_2)


if __name__ == "__main__":
    omni = touch_syn()
    omni.PID()


## 우선 이렇게 서로 feedback 켜놓고 작동하고.. 그 다음에 나중에 마스터 slave 정해서 마스터 항에 대해서는 조금 PID gain 값을 약하게 조정하는 느낌으로 설계하면 더 도움이 될 수도 있을듯 ! ! ! ! ! ! ! ! ! ! ! !
### omni_state_dual 고치고..... 이 파일 작동시켜보고,, master 판단해서 계산하는 거 하면 좋을듯 ! ###