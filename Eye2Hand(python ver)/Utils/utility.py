import math
import numpy as np
import scipy.io as io
import yaml


def kabsch(q, p):
    """
    Description: 坐标系q到p的旋转矩阵R和平移向量t
    :param q: src coordinate system
    :param p: target coordinate system
    :return:  Rt_{q2p}
    """
    # P和q是两个点集的坐标。
    # p和q应该是尺寸（N，3）的二维数组，其中N是点数。
    # 每个数组的3列应分别包含每个点的x、y和z坐标。

    # 计算两个点集的质心。
    centroid_p = np.mean(p, axis=0)
    centroid_q = np.mean(q, axis=0)

    # 通过减去它们的质心来使点集居中。
    p_centered = p - centroid_p
    q_centered = q - centroid_q

    # 计算中心点集的协方差矩阵。
    cov = p_centered.T.dot(q_centered)

    # 计算协方差矩阵的奇异值分解。
    U, S, V = np.linalg.svd(cov)

    # 通过取U和V矩阵的点积来计算旋转矩阵。
    R = U.dot(V)

    # 通过取质心的差异来计算平移矢量
    # 两个点集，并将其乘以旋转矩阵。
    T = centroid_p - R.dot(centroid_q)

    return R, T


def cal_rot_vec(Px_1, Px_2):
    r_1 = np.mean(Px_1 - Px_2, axis=1)
    r_1 = r_1 / np.linalg.norm(r_1)
    return r_1


def angle2dcm(eularAngle):
    """
    欧拉角转旋转矩阵
    :param eularAngle: 欧拉角
    :return: 旋转矩阵
    """
    alpha, beta, gamma = eularAngle[0], eularAngle[1], eularAngle[2]
    R = np.zeros((3, 3))
    R[0, 0] = math.cos(alpha) * math.cos(beta)
    R[0, 1] = math.cos(beta) * math.sin(alpha)
    R[0, 2] = -math.sin(beta)
    R[1, 0] = math.sin(gamma) * math.sin(beta) * math.cos(alpha) - math.cos(gamma) * math.sin(alpha)
    R[1, 1] = math.sin(gamma) * math.sin(beta) * math.sin(alpha) + math.cos(gamma) * math.cos(alpha)
    R[1, 2] = math.sin(gamma) * math.cos(beta)
    R[2, 0] = math.cos(gamma) * math.sin(beta) * math.cos(alpha) + math.sin(gamma) * math.sin(alpha)
    R[2, 1] = math.cos(gamma) * math.sin(beta) * math.sin(alpha) - math.sin(gamma) * math.cos(alpha)
    R[2, 2] = math.cos(gamma) * math.cos(beta)

    return R


def dcm2angle(R):
    """
    description:
        将旋转矩阵转换成欧拉角（弧度），除了排列顺序之外（x和z的顺序），结果和matlab的一致
    :param R: 旋转矩阵
    :return:角度值 - r1~r3，分别对应pitch、yall、roll
    """

    r1 = math.atan2(R[0, 1], R[0, 0])
    r21 = np.clip(-R[0, 2], -1, 1)
    r2 = math.asin(r21)
    r3 = math.atan2(R[1, 2], R[2, 2])
    return np.array([r1, r2, r3])


def Rt2T(R, t):
    btm = np.array([0., 0., 0., 1.])
    T = np.vstack((np.hstack((R, t.reshape(3, 1))), btm))
    return T


def data_load(mat_file_path, keyword_list):
    data_dict = {}
    for kw in keyword_list:
        data = io.loadmat(mat_file_path)[kw]
        data_dict[kw] = data
    return data_dict


def pose2T(pose):
    """
    description:
        读取机械臂参数，返回末端坐标系到基坐标系的变换关系T_E2B
    :param pose: 机械臂参数
    :return: 末端坐标系到基坐标系的变换关系T_E2B
    """
    Px = pose[0]
    Py = pose[1]
    Pz = pose[2]
    R_e2b = angle2dcm(pose[-1:-4:-1])
    t_e2b = np.array([Px, Py, Pz]).reshape(3, 1)  # 末端位姿相对基坐标原点的平移
    T_e2b = Rt2T(R_e2b.T, t_e2b)
    return T_e2b


def T2pose(T):
    """

    :param T:
    :return: pose [Px, Py, Pz, Rx, Ry, Rz]
    """
    R = T[:3, :3]
    t = T[:3, 3]
    angles = dcm2angle(R)
    angles %= 2 * np.pi
    angles[0] = angles[0] - 2 * np.pi if angles[0] > np.pi else angles[0]
    angles[1] = angles[1] - 2 * np.pi if angles[1] > np.pi else angles[1]
    angles[2] = angles[2] - 2 * np.pi if angles[2] > np.pi else angles[2]
    pose = np.hstack((t, angles[::-1]))
    return pose


def saveToYaml(mat, file_name):
    with open(file_name, 'w') as f:
        yaml.dump(mat.tolist(), f)
    print("saved {}.".format(file_name))


def loadFromYaml(file_name):
    with open(file_name) as f:
        loaded = yaml.load(f, Loader=yaml.FullLoader)
    mat = np.array(loaded)
    return mat
