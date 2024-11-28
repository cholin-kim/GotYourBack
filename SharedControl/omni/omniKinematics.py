import numpy as np
import omni.omniVar as omniVar
# import omniVar as omniVar
import rospy

class omniKinematics:
    @classmethod
    def DH_transform(cls, dhparams):  # stacks transforms of neighbor frame, following the modified DH convention
        Ts = [np.array([[np.cos(theta), -np.sin(theta), 0, a],
                        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),
                         -np.sin(alpha) * d],
                        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha),
                         np.cos(alpha) * d],
                        [0, 0, 0, 1]]) for [alpha, a, d, theta] in dhparams]
        return Ts

    @classmethod
    def fk(cls, joints):
        """
        joints = (6,)
        Ts = (Tb1, T12, T23, ...)
        """
        Ts = omniKinematics.DH_transform(omniVar.dhparam(joints))  # Tb1, T12, T23, ...

        # Never use multi_dot for speed-up. It slows down the computation A LOT! Use "for-loop" instead.
        Tbi = np.eye(4)
        Tbs = []
        for T in Ts:
            # Tbe = Tbe @ T
            Tbi = Tbi.dot(T)
            Tbs.append(Tbi)

        # Tbe = np.linalg.multi_dot(Ts)   # from base to end-effector
        # Tbs = np.array(
        #     [np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, len(Ts) + 1)])  # Tb1, Tb2, Tb3, ...
        # Tbs[-1]: from base to end effector
        return Tbs, Ts

from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
import time
import numpy as np

joints = []
def callback_omni(data):
    global joints
    joints = data.position

if __name__ == "__main__":
    rospy.init_node('omni_fk_test', anonymous=True)
    rospy.Subscriber('/omni1/joint_states', JointState, callback_omni)
    from scipy.spatial.transform import Rotation as R
    while True:
        if len(joints) != 0:
            T = np.array(omniKinematics.fk(joints))[0][-1]
            # print(T[:3, 3])
            print(R.from_matrix(T[:3, :3]).as_euler('ZYX', degrees=True))
        time.sleep(0.01)
