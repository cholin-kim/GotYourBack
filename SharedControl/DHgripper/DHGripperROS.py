import rospy
from dh_gripper_msgs.msg import GripperCtrl
# from omni_msgs.msg import OmniButtonEvent
from std_msgs.msg import Int8


class DHGripperROS:
    def __init__(self):
        # Initialize ROS node
        if not rospy.get_node_uri():
            rospy.init_node("dh_robotics_gripper_publisher", anonymous=True)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

        # ROS publisher
        self._set_gripper_pub = rospy.Publisher('/gripper/ctrl', GripperCtrl, latch=True, queue_size=1)
        self.omni_button = rospy.Subscriber('/omni1/button_state', Int8, self.omni_button_cb)
        # self.pub_omni_button = rospy.Publisher('/omni1/button_state', OmniButtonEvent, queue_size=1)

        self.grey_button_flag = 0
        self.white_button_flag = 0
        self.grey_button = 0
        self.white_button = 0

        self.speed = 100
        self.set_gripper(position=1000, speed=self.speed,force=20)
        rospy.sleep(1.0)
        print("gripper initialized")

    def omni_button_cb(self, msg:Int8):
        self.grey_button = msg.data
        # self.white_button = msg.white_button
        # if self.grey_button: self.grey_button_flag = 1
        # else: self.grey_button_flag = 0
        
        # if self.white_button: self.white_button_flag = 1
        # else: self.white_button_flag = 0

    def gripper_cmd(self):
        if self.grey_button:
            position= 500
            force = 70
            self.set_gripper(position=position, speed=self.speed, force=force, initialize=False)

        elif not self.grey_button:

        # elif not self.grey_button_flag:
            position = 100
            force = 70
            self.set_gripper(position=position, speed=self.speed, force=force, initialize=False)


        # print([self.grey_button_flag, self.white_button_flag])
        

    #         '''
    # position and the force parameters in the below functions are determined heuristically.
    # '''         

    def set_gripper(self, position, speed, force, initialize=False):
        """
        :param position: 0 ~ 1000 (permille)  (from spec, 0~10mm stroke per jaw)
        :param speed: 0 ~ 100 (%)  (from spec, 0.2s opening/closing time)
        :param force: 0 ~ 100 (%)  (from spec, 20~80N gripping force per jaw)
        :param initialize: True/False
        :return:
        """
        msg = GripperCtrl()
        msg.initialize = initialize
        msg.position = position
        msg.speed = speed
        msg.force = force
        self._set_gripper_pub.publish(msg)





if __name__ == "__main__":
    gripper = DHGripperROS()
    position = [0, 1000]
    speed = 100
    force = 20
    # gripper.set_gripper(position=1000, speed=speed, force=force, initialize=False)
    # rospy.sleep(2)
    while True:
        gripper.gripper_cmd()

        # gripper.set_gripper(position=position[0], speed=speed, force=force, initialize=False)
        # rospy.sleep(1.0)
        # gripper.set_gripper(position=position[1], speed=speed, force=force, initialize=False)
        # rospy.sleep(1.0)
        # mode = str(input())
        # if mode == 'r':
        #     gripper.set_gripper_ready()
        #     # rospy.sleep(2.0)
        # elif mode == 'g':
        #     gripper.set_gripper_grasp()
        #     # rospy.sleep(2.0)
        # elif mode == 'q':
        #     gripper.set_gripper(position=1000, speed=speed, force=force, initialize=False)
        #     quit()
        #     # rospy.sleep(2)



