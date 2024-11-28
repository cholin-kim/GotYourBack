import numpy as np

from omni_msgs.msg import OmniFeedback
from omni.omniKinematics import omniKinematics
from panda_kinematics.pandaKinematics import pandaKinematics
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import rospy
from scipy.spatial.transform import Rotation as R

omni_q_pos = None
panda_q_pos = None
def omni_q_cb(msg:JointState):
    global omni_q_pos
    omni_q_pos = np.array(msg.position)
def panda_q_cb(msg:JointState):
    global panda_q_pos
    panda_q_pos = np.array(msg.position)
rospy.init_node('omni_feedback')
pub = rospy.Publisher('/phantom/force_feedback', OmniFeedback, queue_size=1)
rospy.Subscriber('/phantom/joint_states', JointState, queue_size=1, callback=omni_q_cb)
rospy.Subscriber('/fr3/franka_state_controller/joint_states', JointState, queue_size=1, callback=panda_q_cb)
omni_q_pos = np.array(rospy.wait_for_message('/phantom/joint_states', JointState).position)
panda_q_pos = np.array(rospy.wait_for_message('/fr3/franka_state_controller/joint_states', JointState).position)

feedback_msg = OmniFeedback()

omni_kin = omniKinematics()
panda_kin = pandaKinematics()

T_mb2sb = np.eye(4)
T_mb2sb[:3, :3] = R.from_euler('z', -np.pi/2).as_matrix()

T_me2se = np.eye(4)
T_me2se[:3, :3] = R.from_euler('YX', [np.pi/2, np.pi]).as_matrix()

kp = np.array([10, 10, 15]) * 10
while not rospy.is_shutdown():
    T_mb2me = omni_kin.fk(omni_q_pos)[0][-1]
    T_sb2se = panda_kin.fk(panda_q_pos)[0][-1]
    ee_offset = [ 0.22052, -0.01779,  0.47752]
    T_sb2se_m = np.copy(T_sb2se)
    T_sb2se_m[:3, -1] -= ee_offset
    T_mb2me_ = T_mb2sb @ T_sb2se_m @ np.linalg.inv(T_me2se)

    ref_pos = T_mb2me_[:3, -1]
    
    force_msg = Vector3()

    pos_err = ref_pos - T_mb2me[:3, -1]
    force_msg.x, force_msg.y, force_msg.z = kp * pos_err

    # force_msg.x = 0
    # force_msg.y = 0
    # force_msg.z = 1
    feedback_msg.force = force_msg
    pub.publish(feedback_msg)


    rospy.sleep(0.01)