import rospy
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import RotationSpline
from scipy.interpolate import CubicSpline
from dynamic_reconfigure import client
from panda_kinematics.pandaKinematics import pandaKinematics


class CartesianCmd:
    def __init__(self, arm='R'):
        if not rospy.get_node_uri():
            rospy.init_node('shared_control_node')
        self.arm = arm
        self.kin = pandaKinematics()

        if arm == 'L':
            self.robot_name = '/panda1'
        elif arm == 'R':
            self.robot_name = '/panda2'

        self.joint_stiffness_param_name = self.robot_name + '/teleop_shared_controller/k_gains'
        self.carte_stiffness_param_name = self.robot_name + '/teleop_shared_controller/cartesian_stiffness'
        self.carte_damping_param_name = self.robot_name + '/teleop_shared_controller/cartesian_damping'
        rospy.Subscriber(self.robot_name + '/franka_state_controller/franka_states', FrankaState,
                         callback=self.franka_state_cb)
        fs = rospy.wait_for_message(self.robot_name + '/franka_state_controller/franka_states', FrankaState)
        # self.T_b2ee = np.array(fs.O_T_EE).reshape(4, 4).T
        self.pub_target_pose = rospy.Publisher('/panda_sim' + self.robot_name + '/shared_target_pose', PoseStamped, queue_size=1)
        # self.pub_target_q = rospy.Publisher('/panda_sim/panda1/shared_target_q', JointState, queue_size=1)
        self.rate = rospy.Rate(100)
        self.dr_client = client.Client(self.robot_name + '/teleop_shared_controllerdynamic_reconfigure_compliance_param_node',
                                       config_callback=self.dr_cb)
        self.stiff_dict = dict()
        self.config_dict = self.dr_client.get_configuration()

    def dr_cb(self, config):
        # rospy.loginfo("Config set to {translational_stiffness}, {translational_damping}, {rotational_stiffness}, {rotational_damping}, {nullspace_stiffness}".format(**config))
        pass

    def franka_state_cb(self, msg: FrankaState):
        # self.T_b2ee = np.array(msg.O_T_EE).reshape(4, 4).T
        pass

    def get_cur_T(self):
        js = rospy.wait_for_message(self.robot_name + '/franka_state_controller/joint_states', JointState)
        self.T_b2ee = self.kin.fk(np.array(js.position))[0][-1]
        return self.T_b2ee

    def get_compliance_params(self):
        self.stiff_dict['trans_stiff'] = self.dr_client.get_configuration()['translational_stiffness']
        self.stiff_dict['trans_damp'] = self.dr_client.get_configuration()['translational_damping']
        self.stiff_dict['ZZZ_trans_stiff'] = self.dr_client.get_configuration()['ZZZ_translational_stiffness']
        self.stiff_dict['ZZZ_trans_damp'] = self.dr_client.get_configuration()['ZZZ_translational_damping']
        self.stiff_dict['rot_stiff'] = self.dr_client.get_configuration()['rotational_stiffness']
        self.stiff_dict['rot_damp'] = self.dr_client.get_configuration()['rotational_damping']
        self.stiff_dict['null_stiff'] = self.dr_client.get_configuration()['nullspace_stiffness']

        return self.stiff_dict
    
    def set_compliance_params(self, compliance_params):
        compliance_params = np.array(list(compliance_params))
        self.config_dict['translational_stiffness'] = compliance_params[0]
        self.config_dict['translational_damping'] = compliance_params[1]
        self.config_dict['ZZZ_translational_stiffness'] = compliance_params[2]
        self.config_dict['ZZZ_translational_damping'] = compliance_params[3]
        self.config_dict['rotational_stiffness'] = compliance_params[4]
        self.config_dict['rotational_damping'] = compliance_params[5]
        self.config_dict['nullspace_stiffness'] = compliance_params[6]
        self.dr_client.update_configuration(self.config_dict)
        

    def set_pose_cmd(self, targ_T, duration=3.0):
        Ts = self.interpolate_pose(targ_T, duration)
        for Tt in Ts:
            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = Tt[:3, -1]
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = R.from_matrix(Tt[:3, :3]).as_quat()
            self.pub_target_pose.publish(ps)
            self.rate.sleep()
    
    def set_pose_cmd_direct(self, targ_T):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = targ_T[:3, -1]
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = R.from_matrix(targ_T[:3, :3]).as_quat()
        self.pub_target_pose.publish(ps)


    def interpolate_pose(self, targ_T, duration):
        t = np.array([0, duration])
        self.T_b2ee = self.get_cur_T()
        cur_p = self.T_b2ee[:3, -1]
        targ_p = targ_T[:3, -1]

        cs = CubicSpline(t, np.array([cur_p, targ_p]), bc_type='clamped')

        cur_r = R.from_matrix(self.T_b2ee[:3, :3])
        targ_r = R.from_matrix(targ_T[:3, :3])

        RS = RotationSpline(t, R.concatenate([cur_r, targ_r]))

        ts = np.linspace(0, duration, int(duration * 100))
        p_traj = cs(ts)
        r_traj = RS(ts).as_matrix()
        Ts = np.zeros((len(p_traj), 4, 4))
        Ts[:, -1, -1] = 1
        Ts[:, :3, -1] = p_traj
        Ts[:, :3, :3] = r_traj
        return Ts


if __name__ == '__main__':
    SC = CartesianCmd(arm='L')
    # targ_T = np.eye(4)
    # targ_T[:3, -1] = [0.4, 0, 0.5]
    # targ_T[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    targ_T = np.copy(SC.get_cur_T())
    targ_T[1, -1] += -0.01
    targ_T[0, -1] += 0.01

    while not rospy.is_shutdown():

        SC.set_pose_cmd(targ_T, duration=5)
        # SC.set_pose_cmd(targ_T_2, duration=0.5)
        # SC.set_pose_cmd(targ_T_3, duration=0.5)
        # SC.set_pose_cmd(targ_T_4, duration=0.5)
        # SC.set_pose_cmd(targ_T, duration=0.5)

        break


        # stiff_dict = SC.get_compliance_param()
        # print(stiff_dict)
        # SC.set_compliance_params(np.array(list(stiff_dict.values())) * .5)
        # stiff_dict = SC.get_compliance_param()
        # print(stiff_dict)

        # carte_stiff = stiff_dict['cartesian_stiff']
        # joint_stiff = stiff_dict['joint_stiff']
        # carte_damp = stiff_dict['cartesian_damping']
        
        # carte_stiff *= 0.5
        # carte_damp *= 0.5
        # SC.set_cartesian_stiffness(carte_stiff)
        # SC.set_cartesian_damping(carte_damp)
        # stiff_dict = SC.get_stifness_param()
        # print(stiff_dict)
        # SC.set_pose_cmd(targ_T, duration=3.0)
        # SC.set_pose_cmd(targ_T_2, duration=1.0)
        # SC.set_pose_cmd(targ_T_3, duration=0.5)
        # break