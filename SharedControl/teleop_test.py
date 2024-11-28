import rospy, time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from omni.omniKinematics import omniKinematics
from panda_kinematics.pandaKinematics import pandaKinematics
import numpy as np
from scipy.spatial.transform import Rotation as R
from omni_msgs.msg import OmniButtonEvent
from SharedControl.phidget.FootPed import FootPed

omni_pose_T = []
target_pose_T = np.eye(4)
q = []
button = []



def callback_omni(data):
    global omni_pose_T, q
    q = np.array(data.position)
    omni_pose_T = omniKinematics.fk(q)[0][-1]

def callback_omni_button(data):
    global button
    button = [data.grey_button, data.white_button]

assem_joint = []
def callback_assem_joint(data):
    global assem_joint
    assem_joint = data.position

cam_joint = []
def callback_cam_joint(data):
    global cam_joint
    cam_joint = data.position

target_pose = []
def callback_target_pose(data):
    global target_pose, target_pose_T
    target_pose = np.array(data.data)
    target_pose_T[:3, :3] = R.from_quat(target_pose[3:]).as_matrix()
    target_pose_T[:3, -1] = target_pose[:3]


def rot_x(rad):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('x', rad).as_matrix()
    return T

def rot_y(rad):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('y', rad).as_matrix()
    return T

def rot_z(rad):
    T = np.eye(4)
    T[:3, :3] = R.from_euler('z', rad).as_matrix()
    return T



rospy.init_node('coppeliasim_ADD_teleop')
pub_pose = rospy.Publisher('/panda_targ_pose', Float32MultiArray, queue_size=1)
pub = rospy.Publisher('/panda_joint_cmd_pid_send', JointState, queue_size=1)
rospy.Subscriber('/panda_joint_state', JointState, callback_assem_joint)
rospy.Subscriber('/panda_cam_joint_state', JointState, callback_cam_joint)
rospy.Subscriber('/omni/omni/joint_states', JointState, callback_omni)
rospy.Subscriber('/omni/omni/button', OmniButtonEvent, callback_omni_button)
rospy.Subscriber('/target_pose', Float32MultiArray, callback_target_pose)

rospy.wait_for_message('/panda_joint_state', JointState)

print('Click any omni button to start task.')
while True:
    if len(button) != 0:
        button = [0, 0]
        break
    time.sleep(0.01)

T_bb = rot_z(-np.pi / 2)
T_ee = rot_y(-np.pi / 2) @ rot_z(-np.pi / 2)

dpos_cam = np.zeros(3)
drot_cam = np.eye(3)
dpos = np.zeros(3)
drot = np.eye(3)

st = time.time_ns()
print('start')

ped = FootPed()

T_sb_cb = np.eye(4)
T_sb_cb[:3, -1] = [0.09999996423721313, -0.6000000238418579, 0.0]
T_sb_cb[:3, :3] = R.from_quat([0.0, 0.0, 0.2588190734386444, 0.9659258723258972]).as_matrix()

T_cb_sb = T_sb_cb.T
cmd = JointState()
joint_state = [-0.17409443855285645, -1.5254404544830322, -0.00015020370483398438, -3.072023391723633, 0.0010726451873779297, 2.7454140186309814, -2.384185791015625e-07]
while True:

    cmd_cam = JointState()
    cam_joint_topic = rospy.resolve_name('/panda_cam_joint_state')
    cam_joint_state = rospy.wait_for_message(cam_joint_topic, JointState)
    cam_joint_names = cam_joint_state.name
    cam_joint_position = list(cam_joint_state.position)

    ee_T_cam = pandaKinematics.fk(cam_joint_position)[0][-1]
    pose_cmd_cam = T_bb @ omni_pose_T @ T_ee
    pose_cmd_cam[0, 3] += 0.1
    pose_cmd_cam[2, 3] += 0.2


    ########################################################################################################################
    # joint_topic = rospy.resolve_name('/panda_joint_state')

    ee_T = pandaKinematics.fk(assem_joint)[0][-1]
    pose_cmd = T_bb @ omni_pose_T @ T_ee

    # pose_cmd = np.linalg.inv(ee_T_cam) @ T_cb_sb @ pose_cmd @ rot_x(np.pi/2)
    # ee_T = np.linalg.inv(ee_T_cam) @ T_cb_sb @ ee_T

    print(joint_state[0])
    if ped.get_pedal_state()[0] == 1:
        pose_cmd[:3, -1] += dpos
        pose_cmd[:3, :3] = pose_cmd[:3, :3] @ drot
        # pose_cmd[0, 3] += 0.1
        # pose_cmd[2, 3] += 0.2
        joint_state = pandaKinematics.ik(pose_cmd, k=0.5)
        # cmd.data = pose_cmd.T.flatten()
        # print(pose_cmd)
        cmd.position = joint_state
        pub.publish(cmd)

    elif ped.get_pedal_state()[0] == 0:
        dpos = ee_T[:3, -1] - pose_cmd[:3, -1]
        drot = np.linalg.inv(pose_cmd[:3, :3]) @ ee_T[:3, :3]

    ########################################################################################################################

    # if ped.get_pedal_state()[2] == 1:
    #     pose_cmd_cam[:3, -1] += dpos_cam
    #     pose_cmd_cam[:3, :3] = pose_cmd_cam[:3, :3] @ drot_cam
    #     joint_position_cam = pandaKinematics.ik(pose_cmd_cam, q0=cam_joint_position, k=0.5)
    #     cmd_cam.name = cam_joint_names
    #     cmd_cam.position = joint_position_cam
    #     pub_cam.publish(cmd_cam)
    #
    # elif ped.get_pedal_state()[2] == 0:
    #     dpos_cam = ee_T_cam[:3, -1] - pose_cmd_cam[:3, -1]
    #     drot_cam = np.linalg.inv(pose_cmd_cam[:3, :3]) @ ee_T_cam[:3, :3]

    ########################################################################################################################

    # if ped.get_pedal_state()[1] == 1:
    #     targ_T = np.eye(4)
    #     # targ_T[:3, :3] = ee_T[:3, :3]
    #     targ_T[:3, -1] = target_pose_T[:3, -1]
    #     targ_T[:3, :3] = target_pose_T[:3, :3] @ R.from_euler('yz', [np.pi, np.pi/2]).as_matrix()
    #     joint_position = pandaKinematics.ik(targ_T, q0=cam_joint_position, k=0.5)
    #     cmd.name = joint_names
    #     cmd.position = joint_position
    #     pub.publish(cmd)

    time.sleep(0.001)