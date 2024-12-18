import rospy
import numpy as np
from sensor_msgs.msg import JointState
from omniKinematics import omniKinematics
from omni_msgs.msg import OmniFeedback
import matplotlib.pyplot as plt


class touch_syn():
    def __init__(self):
        rospy.Subscriber('/omni1/joint_states', JointState, self.omni_cb_1)
        rospy.Subscriber('/omni2/joint_states', JointState, self.omni_cb_2)

        self.cur_pose_1 = np.array(rospy.wait_for_message('/omni1/joint_states', JointState).position)
        self.cur_pose_2 = np.array(rospy.wait_for_message('/omni2/joint_states', JointState).position)

        self.pub_1 = rospy.Publisher('/omni1/force_feedback', OmniFeedback, queue_size=1)
        self.pub_2 = rospy.Publisher('/omni2/force_feedback', OmniFeedback, queue_size=1)
        self.feedback = OmniFeedback()

        self.master_1 = False
        self.master_2 = False

        self.kin = omniKinematics()
        a, _ = self.kin.fk(self.cur_pose_1)
        b, _ = self.kin.fk(self.cur_pose_2)
        self.prev_position_1 = a[-1][:3, 3]
        self.prev_position_2 = b[-1][:3, 3]

        # /--- 추가됨 #
        self.e_i = np.zeros(3)
        self.e_p = np.zeros(3)
        self.time_slice = 1 # 알아내야 하는 값
        #  --- ---/ #

    def omni_cb_1(self, msg:JointState):
        self.cur_pose_1 = np.array(msg.position)
        self.check_master_1(self.cur_pose_1)

    def omni_cb_2(self, msg:JointState):
        self.cur_pose_2 = np.array(msg.position)
        self.check_master_2(self.cur_pose_2)

    def check_master_1(self, pose_1):
        current_p, _ = self.kin.fk(pose_1)
        current = current_p[-1][:3, 3]
        movement_1 = np.linalg.norm(self.prev_position_1 - current)
        if movement_1 > 0.5:
            if not self.master_2:
                self.master_1 = True
                self.master_2 = False

        else: # 너무 쉽게 바뀌면 buffer 넣기
            self.master_2 = False
            self.master_1 = True
            # /--- 추가됨 #
            self.e_i = np.zeros(3)
            self.e_p = np.zeros(3)
            #  --- ---/ #

        self.prev_position_1 = current

    def check_master_2(self, pose_2):
        current_p, _ = self.kin.fk(pose_2)
        current = current_p[-1][:3, 3]
        movement_2 = np.linalg.norm(self.prev_position_2 - current)
        if movement_2 > 0.5:
            if not self.master_1:
                self.master_2 = True
                self.master_1 = False

        else:
            self.master_1 = False
            self.master_2 = True
            # /--- 추가됨 #
            self.e_i = np.zeros(3)
            self.e_p = np.zeros(3)
            #  --- ---/ #

        self.prev_position_2 = current

    def PID(self):  # 대부분 바꿈
        desired = 0
        current = 0

        # parameters
        x_K_p = 12
        x_K_i = 0
        x_K_d = 5

        y_K_p = 12
        y_K_i = 0
        y_K_d = 5

        z_K_p = 12
        z_K_i = 0
        z_K_d = 5

        # Motor 힘에 따른 parameters
        # constant_x = 2
        # constant_y = 5
        # constant_z = 15

        rospy.Rate(1000)

        while not rospy.is_shutdown():
            a, _ = self.kin.fk(self.cur_pose_1)
            b, _ = self.kin.fk(self.cur_pose_2)

            if self.master_1 == True:
                desired = a[-1][:3, 3]
                current = b[-1][:3, 3]
            else:
                desired = b[-1][:3, 3]
                current = a[-1][:3, 3]


            e_p = desired - current
            self.e_i += e_p
            e_d = (e_p - self.e_p)
            self.e_p = e_p

            control_x = (x_K_p * e_p[0] + x_K_i * self.e_i[0] + x_K_d * e_d[0])
            control_y = (y_K_p * e_p[1] + y_K_i * self.e_i[1] + y_K_d * e_d[1])
            control_z = (z_K_p * e_p[2] + z_K_i * self.e_i[2] + z_K_d * e_d[2])

            self.feedback.force.x = control_x
            self.feedback.force.y = control_y
            self.feedback.force.z = control_z

            if self.master_1 == False:
                self.pub_1.publish(self.feedback)
            else:
                self.pub_2.publish(self.feedback)


if __name__ == '__main__':
    omni = touch_syn()
    omni.PID()