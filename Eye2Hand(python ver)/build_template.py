import numpy as np
import math
import scipy.io as io
from Utils.utility import kabsch, saveToYaml, loadFromYaml  # 使用kabsch函数计算旋转平移矩阵


def isRotationMatrix(R):
    """
    description:
        检查输入的矩阵是否符合欧拉角的条件,即是否为正交阵
    :param R: 旋转矩阵
    :return: n
    """
    R_trans = np.transpose(R)
    shouldBeIdentity = np.dot(R_trans, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    """
    description:
        将旋转矩阵转换成欧拉角（弧度），除了排列顺序之外（x和z的顺序），结果和matlab的一致
    :param R: 旋转矩阵
    :return:角度值 - r1~r3，分别对应pitch、yall、roll
    """
    assert (isRotationMatrix(R))

    r1 = math.atan2(R[0, 1], R[0, 0])
    r21 = np.clip(-R[0, 2], -1, 1)
    r2 = math.asin(r21)
    r3 = math.atan2(R[1, 2], R[2, 2])
    return np.array([r1, r2, r3])


def eulerAnglesToRotationMatrix(alpha, beta, gamma):
    """
    description:
        欧拉角转旋转矩阵
    :param alpha:
    :param beta:
    :param gamma:
    :return:
    """
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


def calp2line(a, b, p):
    """
    description:
        计算三维点P到AB组成的直线的距离
    :param:
        3Dimension A，B，P
    :return:
        Distance
    """

    A_x = a[0]
    A_y = a[1]
    A_z = a[2]
    B_x = b[0]
    B_y = b[1]
    B_z = b[2]
    P_x = p[0]
    P_y = p[1]
    P_z = p[2]
    ##
    # abc = abs(b - c)
    # di s2 = scipy.linalg.norm(abc, 2)
    ##
    P = math.sqrt((math.pow((A_x - B_x), 2) + math.pow((A_y - B_y), 2) + math.pow((A_z - B_z), 2)))
    B = math.sqrt((math.pow((A_x - P_x), 2) + math.pow((A_y - P_y), 2) + math.pow((A_z - P_z), 2)))
    A = math.sqrt((math.pow((P_x - B_x), 2) + math.pow((P_y - B_y), 2) + math.pow((P_z - B_z), 2)))
    cosp = (math.pow(A, 2) + math.pow(B, 2) - math.pow(P, 2)) / (2 * A * B)
    sinp = math.sqrt(1 - math.pow(cosp, 2))
    return A * B * sinp / P


def calTriangle(a, b, c):
    """
    description:
        计算三个三维点组成的三角形面积。面积越大的三角形所对应的向量的叉积的长度也越大。
    :param a: 顶点 a
    :param b: 顶点 b
    :param c: 顶点 c
    :return:
        area 三角形面积
    """

    v1 = b - a
    v2 = c - a
    cross_product = np.cross(v1, v2)  # 计算向量的叉积
    area = 0.5 * np.linalg.norm(cross_product)
    return area


def orderTriangle(Pt_init):
    """
    使用三角形法则构造模板坐标
    :param Pt_init: 3xM，检测到的点集
    :return: Pt_temp: 3xM，基于规则得到的模板坐标
    """
    Pt_num = Pt_init.shape[-1]
    p_idx = []  # 记录顺序
    # 计算四个小球的中心点，确定p0
    center = np.mean(Pt_init, axis=1)  # 3xM->3x1
    dist_center = np.linalg.norm(np.expand_dims(center, axis=1) - Pt_init, axis=0)
    p0_idx = np.argmin(dist_center, axis=0)
    p_idx.append(p0_idx)
    # 寻找距离p0最远的p1
    dist_p0 = np.linalg.norm(Pt_init[:, p0_idx:p0_idx + 1] - Pt_init, axis=0)
    p1_idx = np.argmax(dist_p0, axis=0)
    p_idx.append(p1_idx)
    dist_p0p1 = [0] * Pt_num
    for i in range(Pt_num):
        if i not in p_idx:
            dist_p0p1[i] = calp2line(Pt_init[:, p_idx[0]], Pt_init[:, p_idx[1]], Pt_init[:, i])
    # 从小到大排序，输出逆序即为最大距离索引
    p_others_idx = [i[0] for i in sorted(enumerate(dist_p0p1), key=lambda x: x[1])]
    # 添加剩余点索引
    for k in range(Pt_num - 2):
        p_idx.append(p_others_idx[-1 - k])
    Pt_order = np.zeros(Pt_init.shape)
    for i, idx in enumerate(p_idx):
        Pt_order[:, i] = Pt_init[:, idx]  # 列赋值
    return Pt_order


class template:
    """
    description:
        建造针尖模板坐标系。
    """

    def __init__(self):
        self.Pt_temp = None
        self.Pt_init = None
        self.Pt_num = None
        self.Fig_N = None

    def setInitTemp(self, temp_init):
        # 强制形状为3xM
        if temp_init.shape[0] > 3:
            temp_init = temp_init.T
        assert temp_init.shape[0] == 3 and temp_init.shape[1] >= 3
        self.Pt_init = temp_init
        self.Pt_num = temp_init.shape[-1]
        if np.linalg.norm(self.Pt_init[:, 0]) > 1.0:  # 判断此时模板是否标准
            self.Pt_init = orderTriangle(temp_init)
            self.buildStandardTemp()
        else:
            self.Pt_temp = temp_init

    def getTemp(self):
        return self.Pt_temp

    def optTemp(self, measure3d):
        """
        模板优化，调用optProc完成
        :param measure3d: Nx3xM, 相机坐标系下不同位置的标识球数据(N组，每组M点，每点3D坐标)
        :return:
        """
        # 采集数据的张数
        self.Fig_N = len(measure3d)
        # 基于采集到的数据优化模板
        self.optProc(measure3d)
        saveToYaml(self.Pt_temp, 'temp.yaml')

    def buildStandardTemp(self):
        """
        构造符合三角形法则的标准模板坐标：
        |0 a b d ...|
        |0 0 c e ...|
        |0 0 0 f ...|
        """

        a = np.linalg.norm(self.Pt_init[:, 0] - self.Pt_init[:, 1], 2)
        c = calp2line(self.Pt_init[:, 0], self.Pt_init[:, 1], self.Pt_init[:, 2])
        b = math.sqrt(np.linalg.norm(self.Pt_init[:, 0] - self.Pt_init[:, 2], 2) ** 2 - c ** 2)
        # print(a, b, c)
        # 对P1,P2,P3 相对于P0进行偏移
        self.Pt_init -= self.Pt_init[:, 0:1]
        """
        计算当前点集self.Pt_init->标准模板self.Pt_temp的旋转矩阵R
        因为仅平移后，原点重合，但其他点尚未重合，可根据几何关系求解一个R，[d;e;f]后续的点（若存在）仅需左乘R实现标准化！！！
                   | 0 a b d ...|        | 0                       |
         Pt_temp = | 0 0 c e ...| ,  and | 0 p1-p0 p2-p0 p3-p0 ... | = R*Pt_temp
                   | 0 0 0 f ...|        | 0                       |
        基于a,b,c和p1-p0 p2-p0的值便能求解R，再用已知的p3-p0，...，p（M-1）-p0求解f后续（若存在）的参数
        Pt_temp = R_inv*[ 0 p1-p0 p2-p0 p3-p0 ... ]
        """
        r00 = self.Pt_init[0][1] / a  # 注意，行代表xyz，列代表点索引
        r01 = self.Pt_init[1][1] / a
        r02 = self.Pt_init[2][1] / a
        r10 = (self.Pt_init[0][2] - b * r00) / c
        r11 = (self.Pt_init[1][2] - b * r01) / c
        r12 = (self.Pt_init[2][2] - b * r02) / c
        # Outer product
        r20 = r01 * r12 - r11 * r02
        r21 = r02 * r10 - r00 * r12
        r22 = r00 * r11 - r01 * r10
        R = np.array([[r00, r10, r20], [r01, r11, r21], [r02, r12, r22]])
        R_inv = np.linalg.inv(R)  # R的逆
        self.Pt_temp = np.matmul(R_inv, self.Pt_init)  # 矩阵相乘
        # 下三角强制置0
        self.Pt_temp[1][1] = 0
        self.Pt_temp[2][1:3] = 0
        print("initial template = \n", self.Pt_temp)

    def optProc(self, measure3d):
        """
        模板优化子函数
        :param measure3d: Nx3xM, 相机坐标系下不同位置的标识球数据(N组，每组M点，每点3D坐标)
        """
        # 3*self.Pt_num-6表示模板未知参数a~f，...
        # 6*self.Fig_N表示每组测量数据的欧拉角和平移向量
        Params = np.zeros(3 * self.Pt_num - 6 + 6 * self.Fig_N)
        # Params拼接顺序按a~f...|alpha0 beta0 gamma0 x0 y0 z0 ... alpha(N-1) beta(N-1) gamma(N-1) x(N-1) y(N-1) z(N-1)
        Params[0] = self.Pt_temp[0][1] + 1  # a，添加扰动
        Params[1] = self.Pt_temp[0][2] + 1  # b
        Params[2] = self.Pt_temp[1][2] - 1  # c
        # 后续部分不确定个数，遍历赋值
        for i in range(3, 3 * self.Pt_num - 6):
            # i%3取行号；i//3+2取列号，增加下三角零阵的偏置2
            Params[i] = self.Pt_temp[i % 3][i // 3 + 2] - 1
        # 计算每组点的R和T并填充
        for k in range(self.Fig_N):
            R, T = kabsch(self.Pt_temp.T, measure3d[k].T)  # 输入矩阵大小为Mx3, Rt_{temp2cam}
            # When using dcm2angle in matlab to calculate Euler angles
            # you need to transpose the rotation matrix
            Params[6 * k + 3 * self.Pt_num - 6:6 * k + 3 * self.Pt_num - 3] = rotationMatrixToEulerAngles(R.T) + \
                                                                              np.array([0.1, -0.1, 0.1])
            Params[6 * k + 3 * self.Pt_num - 3:6 * k + 3 * self.Pt_num] = T + np.array([1, -1, 1])

        def _calGradP():
            """
            内部函数，迭代求优
            """
            # 构造para1和para2
            para1, para2 = Params[:6], Params[6:]
            Q = np.zeros(3 * self.Pt_num * self.Fig_N)
            for i in range(self.Fig_N):
                for j in range(self.Pt_num):
                    Q[12 * i + 3 * j: 12 * i + 3 * j + 3] = measure3d[i, :, j]  # 第i组第j点的3d坐标
            S = para1.shape[0]
            Q_err = np.zeros((3 * self.Pt_num * self.Fig_N, 1))
            J_err = np.zeros((3 * self.Pt_num * self.Fig_N, S + 6 * self.Fig_N))
            # 构造Q_err和J_err
            for i in range(self.Fig_N):
                # Row: 3~8, col: 0~2
                J_err[3 * self.Pt_num * i + 3, 0] = math.cos(para2[6 * i]) * math.cos(para2[6 * i + 1])
                J_err[3 * self.Pt_num * i + 4, 0] = math.cos(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.sin(
                    para2[6 * i + 2]) - \
                                                    math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i])
                J_err[3 * self.Pt_num * i + 5, 0] = math.sin(para2[6 * i]) * math.sin(para2[6 * i + 2]) + \
                                                    math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) * math.sin(
                    para2[6 * i + 1])
                J_err[3 * self.Pt_num * i + 6, 1] = math.cos(para2[6 * i]) * math.cos(para2[6 * i + 1])
                J_err[3 * self.Pt_num * i + 6, 2] = math.cos(para2[6 * i + 1]) * math.sin(para2[6 * i])
                J_err[3 * self.Pt_num * i + 7, 1] = math.cos(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.sin(
                    para2[6 * i + 2]) - \
                                                    math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i])
                J_err[3 * self.Pt_num * i + 7, 2] = math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) + \
                                                    math.sin(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.sin(
                    para2[6 * i + 2])
                J_err[3 * self.Pt_num * i + 8, 1] = math.sin(para2[6 * i]) * math.sin(para2[6 * i + 2]) + \
                                                    math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) * math.sin(
                    para2[6 * i + 1])
                J_err[3 * self.Pt_num * i + 8, 2] = math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i]) * math.sin(
                    para2[6 * i + 1]) - \
                                                    math.cos(para2[6 * i]) * math.sin(para2[6 * i + 2])

                # Row: 0~8, col: S~S + 5
                J_err[3 * self.Pt_num * i, S + 6 * i + 3] = 1
                J_err[3 * self.Pt_num * i + 1, S + 6 * i + 4] = 1
                J_err[3 * self.Pt_num * i + 2, S + 6 * i + 5] = 1

                J_err[3 * self.Pt_num * i + 3, S + 6 * i] = -math.cos(para2[6 * i + 1]) * math.sin(para2[6 * i]) * \
                                                            para1[0]
                J_err[3 * self.Pt_num * i + 3, S + 6 * i + 1] = -math.cos(para2[6 * i]) * math.sin(para2[6 * i + 1]) * \
                                                                para1[0]
                J_err[3 * self.Pt_num * i + 3, S + 6 * i + 3] = 1

                J_err[3 * self.Pt_num * i + 4, S + 6 * i] = -para1[0] * (
                        math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) + \
                        math.sin(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.sin(para2[6 * i + 2]))
                J_err[3 * self.Pt_num * i + 4, S + 6 * i + 1] = math.cos(para2[6 * i]) * math.cos(
                    para2[6 * i + 1]) * math.sin(para2[6 * i + 2]) * para1[0]
                J_err[3 * self.Pt_num * i + 4, S + 6 * i + 2] = para1[0] * (
                        math.sin(para2[6 * i]) * math.sin(para2[6 * i + 2]) + \
                        math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i + 1]))
                J_err[3 * self.Pt_num * i + 4, S + 6 * i + 4] = 1

                J_err[3 * self.Pt_num * i + 5, S + 6 * i] = para1[0] * (
                        math.cos(para2[6 * i]) * math.sin(para2[6 * i + 2]) - \
                        math.sin(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.cos(para2[6 * i + 2]))
                J_err[3 * self.Pt_num * i + 5, S + 6 * i + 1] = math.cos(para2[6 * i]) * math.cos(
                    para2[6 * i + 1]) * math.cos(para2[6 * i + 2]) * para1[0]
                J_err[3 * self.Pt_num * i + 5, S + 6 * i + 2] = para1[0] * (
                        math.sin(para2[6 * i]) * math.cos(para2[6 * i + 2]) - math.cos(para2[6 * i]) * math.sin(
                    para2[6 * i + 2]) * math.sin(para2[6 * i + 1]))
                J_err[3 * self.Pt_num * i + 5, S + 6 * i + 5] = 1

                J_err[3 * self.Pt_num * i + 6, S + 6 * i] = math.cos(para2[6 * i]) * math.cos(para2[6 * i + 1]) * para1[
                    2] - math.cos(para2[6 * i + 1]) * math.sin(para2[6 * i]) * para1[1]
                J_err[3 * self.Pt_num * i + 6, S + 6 * i + 1] = -math.cos(para2[6 * i]) * math.sin(para2[6 * i + 1]) * \
                                                                para1[1] - math.sin(para2[6 * i]) * math.sin(
                    para2[6 * i + 1]) * para1[2]
                J_err[3 * self.Pt_num * i + 6, S + 6 * i + 3] = 1

                J_err[3 * self.Pt_num * i + 7, S + 6 * i] = -para1[1] * (
                        math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) +
                        math.sin(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.sin(para2[6 * i + 2])) - \
                                                            para1[2] * (math.cos(para2[6 * i + 2]) * math.sin(
                    para2[6 * i]) -
                                                                        math.cos(para2[6 * i]) * math.sin(
                            para2[6 * i + 1]) * math.sin(para2[6 * i + 2]))
                J_err[3 * self.Pt_num * i + 7, S + 6 * i + 1] = math.cos(para2[6 * i]) * math.cos(
                    para2[6 * i + 1]) * math.sin(para2[6 * i + 2]) * para1[1] + math.cos(para2[6 * i + 1]) * math.sin(
                    para2[6 * i]) * math.sin(para2[6 * i + 2]) * para1[2]
                J_err[3 * self.Pt_num * i + 7, S + 6 * i + 2] = para1[1] * (
                        math.sin(para2[6 * i]) * math.sin(para2[6 * i + 2]) +
                        math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i + 1])) - \
                                                                para1[2] * (math.cos(para2[6 * i]) * math.sin(
                    para2[6 * i + 2]) - \
                                                                            math.cos(para2[6 * i + 2]) * math.sin(
                            para2[6 * i]) * math.sin(para2[6 * i + 1]))
                J_err[3 * self.Pt_num * i + 7, S + 6 * i + 4] = 1

                J_err[3 * self.Pt_num * i + 8, S + 6 * i] = para1[1] * (
                        math.cos(para2[6 * i]) * math.sin(para2[6 * i + 2]) - \
                        math.sin(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.cos(para2[6 * i + 2])) + \
                                                            para1[2] * (math.sin(para2[6 * i]) * math.sin(
                    para2[6 * i + 2]) + \
                                                                        math.cos(para2[6 * i]) * math.cos(
                            para2[6 * i + 2]) * math.sin(para2[6 * i + 1]))
                J_err[3 * self.Pt_num * i + 8, S + 6 * i + 1] = math.cos(para2[6 * i]) * math.cos(
                    para2[6 * i + 1]) * math.cos(para2[6 * i + 2]) * para1[1] + \
                                                                math.cos(para2[6 * i + 1]) * math.cos(
                    para2[6 * i + 2]) * math.sin(para2[6 * i]) * para1[2]
                J_err[3 * self.Pt_num * i + 8, S + 6 * i + 2] = para1[1] * (
                        math.sin(para2[6 * i]) * math.cos(para2[6 * i + 2]) - \
                        math.cos(para2[6 * i]) * math.sin(para2[6 * i + 2]) * math.sin(para2[6 * i + 1])) - \
                                                                para1[2] * (math.cos(para2[6 * i]) * math.cos(
                    para2[6 * i + 2]) + \
                                                                            math.sin(para2[6 * i]) * math.sin(
                            para2[6 * i + 1]) * math.sin(para2[6 * i + 2]))
                J_err[3 * self.Pt_num * i + 8, S + 6 * i + 5] = 1

                Q_err[3 * self.Pt_num * i] = para2[6 * i + 3] - Q[3 * self.Pt_num * i]
                Q_err[3 * self.Pt_num * i + 1] = para2[6 * i + 4] - Q[3 * self.Pt_num * i + 1]
                Q_err[3 * self.Pt_num * i + 2] = para2[6 * i + 5] - Q[3 * self.Pt_num * i + 2]
                Q_err[3 * self.Pt_num * i + 3] = para2[6 * i + 3] - Q[3 * self.Pt_num * i + 3] + \
                                                 para1[0] * math.cos(para2[6 * i]) * math.cos(para2[6 * i + 1])
                Q_err[3 * self.Pt_num * i + 4] = para2[6 * i + 4] - Q[3 * self.Pt_num * i + 4] - \
                                                 para1[0] * (math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i]) - \
                                                             math.cos(para2[6 * i]) * math.sin(
                            para2[6 * i + 1]) * math.sin(para2[6 * i + 2]))
                Q_err[3 * self.Pt_num * i + 5] = para2[6 * i + 5] - Q[3 * self.Pt_num * i + 5] + \
                                                 para1[0] * (math.sin(para2[6 * i]) * math.sin(para2[6 * i + 2]) + \
                                                             math.cos(para2[6 * i]) * math.cos(
                            para2[6 * i + 2]) * math.sin(para2[6 * i + 1]))
                Q_err[3 * self.Pt_num * i + 6] = para2[6 * i + 3] - Q[3 * self.Pt_num * i + 6] + \
                                                 para1[1] * math.cos(para2[6 * i]) * math.cos(para2[6 * i + 1]) + \
                                                 para1[2] * math.cos(para2[6 * i + 1]) * math.sin(para2[6 * i])
                Q_err[3 * self.Pt_num * i + 7] = para2[6 * i + 4] - Q[3 * self.Pt_num * i + 7] - \
                                                 para1[1] * (math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i]) - \
                                                             math.cos(para2[6 * i]) * math.sin(
                            para2[6 * i + 1]) * math.sin(para2[6 * i + 2])) + \
                                                 para1[2] * (math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) + \
                                                             math.sin(para2[6 * i]) * math.sin(
                            para2[6 * i + 1]) * math.sin(para2[6 * i + 2]))
                Q_err[3 * self.Pt_num * i + 8] = para2[6 * i + 5] - Q[3 * self.Pt_num * i + 8] + para1[1] * (
                        math.sin(para2[6 * i]) * math.sin(para2[6 * i + 2]) + \
                        math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i + 1])) - \
                                                 para1[2] * (math.cos(para2[6 * i]) * math.sin(para2[6 * i + 2]) - \
                                                             math.cos(para2[6 * i + 2]) * math.sin(
                            para2[6 * i]) * math.sin(para2[6 * i + 1]))

                # Row: 9: 3 * self.Pt_num - 1, col: 3~S - 1
                for k in range(4, self.Pt_num + 1):
                    J_err[3 * self.Pt_num * i + 3 * k - 3, 3 * k - 9] = math.cos(para2[6 * i]) * math.cos(
                        para2[6 * i + 1])
                    J_err[3 * self.Pt_num * i + 3 * k - 3, 3 * k - 8] = math.cos(para2[6 * i + 1]) * math.sin(
                        para2[6 * i])
                    J_err[3 * self.Pt_num * i + 3 * k - 3, 3 * k - 7] = -math.sin(para2[6 * i + 1])
                    J_err[3 * self.Pt_num * i + 3 * k - 3, S + 6 * i] = math.cos(para2[6 * i]) * math.cos(
                        para2[6 * i + 1]) * para1[3 * k - 8] - \
                                                                        math.cos(para2[6 * i + 1]) * math.sin(
                        para2[6 * i]) * para1[3 * k - 9]
                    J_err[3 * self.Pt_num * i + 3 * k - 3, S + 6 * i + 1] = -math.cos(para2[6 * i + 1]) * para1[
                        3 * k - 7] - \
                                                                            math.cos(para2[6 * i]) * math.sin(
                        para2[6 * i + 1]) * para1[3 * k - 9] - \
                                                                            math.sin(para2[6 * i]) * math.sin(
                        para2[6 * i + 1]) * para1[3 * k - 8]
                    J_err[3 * self.Pt_num * i + 3 * k - 3, S + 6 * i + 3] = 1

                    J_err[3 * self.Pt_num * i + 3 * k - 2, 3 * k - 9] = math.cos(para2[6 * i]) * math.sin(
                        para2[6 * i + 1]) * math.sin(para2[6 * i + 2]) - \
                                                                        math.cos(para2[6 * i + 2]) * math.sin(
                        para2[6 * i])
                    J_err[3 * self.Pt_num * i + 3 * k - 2, 3 * k - 8] = math.cos(para2[6 * i]) * math.cos(
                        para2[6 * i + 2]) + \
                                                                        math.sin(para2[6 * i]) * math.sin(
                        para2[6 * i + 1]) * math.sin(para2[6 * i + 2])
                    J_err[3 * self.Pt_num * i + 3 * k - 2, 3 * k - 7] = math.cos(para2[6 * i + 1]) * math.sin(
                        para2[6 * i + 2])
                    J_err[3 * self.Pt_num * i + 3 * k - 2, S + 6 * i] = -para1[3 * k - 9] * (
                            math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) + \
                            math.sin(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.sin(para2[6 * i + 2])) - \
                                                                        para1[3 * k - 8] * (math.cos(
                        para2[6 * i + 2]) * math.sin(para2[6 * i]) - \
                                                                                            math.cos(para2[
                                                                                                         6 * i]) * math.sin(
                                para2[6 * i + 1]) * math.sin(para2[6 * i + 2]))
                    J_err[3 * self.Pt_num * i + 3 * k - 2, S + 6 * i + 1] = math.cos(para2[6 * i]) * math.cos(
                        para2[6 * i + 1]) * math.sin(para2[6 * i + 2]) * para1[3 * k - 9] - \
                                                                            math.sin(para2[6 * i + 1]) * math.sin(
                        para2[6 * i + 2]) * para1[3 * k - 7] + \
                                                                            math.cos(para2[6 * i + 1]) * math.sin(
                        para2[6 * i]) * math.sin(para2[6 * i + 2]) * para1[3 * k - 8]
                    J_err[3 * self.Pt_num * i + 3 * k - 2, S + 6 * i + 2] = para1[3 * k - 9] * (
                            math.sin(para2[6 * i]) * math.sin(para2[6 * i + 2]) + \
                            math.cos(para2[6 * i]) * math.cos(para2[6 * i + 2]) * math.sin(para2[6 * i + 1])) - \
                                                                            para1[3 * k - 8] * (math.cos(
                        para2[6 * i]) * math.sin(para2[6 * i + 2]) - \
                                                                                                math.cos(para2[
                                                                                                             6 * i + 2]) * math.sin(
                                para2[6 * i]) * math.sin(para2[6 * i + 1])) + \
                                                                            math.cos(para2[6 * i + 1]) * math.cos(
                        para2[6 * i + 2]) * para1[3 * k - 7]
                    J_err[3 * self.Pt_num * i + 3 * k - 2, S + 6 * i + 4] = 1

                    J_err[3 * self.Pt_num * i + 3 * k - 1, 3 * k - 9] = math.sin(para2[6 * i]) * math.sin(
                        para2[6 * i + 2]) + \
                                                                        math.cos(para2[6 * i]) * math.cos(
                        para2[6 * i + 2]) * math.sin(para2[6 * i + 1])
                    J_err[3 * self.Pt_num * i + 3 * k - 1, 3 * k - 8] = math.cos(para2[6 * i + 2]) * math.sin(
                        para2[6 * i]) * math.sin(para2[6 * i + 1]) - \
                                                                        math.cos(para2[6 * i]) * math.sin(
                        para2[6 * i + 2])
                    J_err[3 * self.Pt_num * i + 3 * k - 1, 3 * k - 7] = math.cos(para2[6 * i + 1]) * math.cos(
                        para2[6 * i + 2])
                    J_err[3 * self.Pt_num * i + 3 * k - 1, S + 6 * i] = para1[3 * k - 9] * (
                            math.cos(para2[6 * i]) * math.sin(para2[6 * i + 2]) - \
                            math.sin(para2[6 * i]) * math.sin(para2[6 * i + 1]) * math.cos(para2[6 * i + 2])) + \
                                                                        para1[3 * k - 8] * (
                                                                                math.sin(para2[6 * i]) * math.sin(
                                                                            para2[6 * i + 2]) + \
                                                                                math.cos(para2[6 * i]) * math.cos(
                                                                            para2[6 * i + 2]) * math.sin(
                                                                            para2[6 * i + 1]))
                    J_err[3 * self.Pt_num * i + 3 * k - 1, S + 6 * i + 1] = math.cos(para2[6 * i]) * math.cos(
                        para2[6 * i + 1]) * math.cos(para2[6 * i + 2]) * para1[3 * k - 9] - \
                                                                            math.cos(para2[6 * i + 2]) * math.sin(
                        para2[6 * i + 1]) * para1[3 * k - 7] + \
                                                                            math.cos(para2[6 * i + 1]) * math.cos(
                        para2[6 * i + 2]) * math.sin(para2[6 * i]) * para1[3 * k - 8]
                    J_err[3 * self.Pt_num * i + 3 * k - 1, S + 6 * i + 2] = para1[3 * k - 9] * (
                            math.sin(para2[6 * i]) * math.cos(para2[6 * i + 2]) - \
                            math.cos(para2[6 * i]) * math.sin(para2[6 * i + 2]) * math.sin(para2[6 * i + 1])) - \
                                                                            para1[3 * k - 8] * (math.cos(
                        para2[6 * i]) * math.cos(para2[6 * i + 2]) + \
                                                                                                math.sin(para2[
                                                                                                             6 * i]) * math.sin(
                                para2[6 * i + 1]) * math.sin(para2[6 * i + 2])) - \
                                                                            math.cos(para2[6 * i + 1]) * math.sin(
                        para2[6 * i + 2]) * para1[3 * k - 7]
                    J_err[3 * self.Pt_num * i + 3 * k - 1, S + 6 * i + 5] = 1

                    Q_err[3 * self.Pt_num * i + 3 * k - 3] = para2[6 * i + 3] - Q[
                        3 * self.Pt_num * i + 3 * k - 3] - math.sin(para2[6 * i + 1]) * para1[3 * k - 7] + \
                                                             math.cos(para2[6 * i]) * math.cos(para2[6 * i + 1]) * \
                                                             para1[3 * k - 9] + \
                                                             math.cos(para2[6 * i + 1]) * math.sin(para2[6 * i]) * \
                                                             para1[3 * k - 8]
                    Q_err[3 * self.Pt_num * i + 3 * k - 2] = para2[6 * i + 4] - Q[3 * self.Pt_num * i + 3 * k - 2] - \
                                                             para1[3 * k - 9] * (math.cos(para2[6 * i + 2]) * math.sin(
                        para2[6 * i]) - \
                                                                                 math.cos(para2[6 * i]) * math.sin(
                                para2[6 * i + 1]) * math.sin(para2[6 * i + 2])) + \
                                                             para1[3 * k - 8] * (math.cos(para2[6 * i]) * math.cos(
                        para2[6 * i + 2]) + \
                                                                                 math.sin(para2[6 * i]) * math.sin(
                                para2[6 * i + 1]) * math.sin(para2[6 * i + 2])) + \
                                                             math.cos(para2[6 * i + 1]) * math.sin(para2[6 * i + 2]) * \
                                                             para1[3 * k - 7]
                    Q_err[3 * self.Pt_num * i + 3 * k - 1] = para2[6 * i + 5] - Q[3 * self.Pt_num * i + 3 * k - 1] + \
                                                             para1[3 * k - 9] * (math.sin(para2[6 * i]) * math.sin(
                        para2[6 * i + 2]) + \
                                                                                 math.cos(para2[6 * i]) * math.cos(
                                para2[6 * i + 2]) * math.sin(para2[6 * i + 1])) - \
                                                             para1[3 * k - 8] * (math.cos(para2[6 * i]) * math.sin(
                        para2[6 * i + 2]) - \
                                                                                 math.cos(para2[6 * i + 2]) * math.sin(
                                para2[6 * i]) * math.sin(para2[6 * i + 1])) + \
                                                             math.cos(para2[6 * i + 1]) * math.cos(para2[6 * i + 2]) * \
                                                             para1[3 * k - 7]

            delta_P = np.dot(-Q_err.T, J_err)
            return delta_P

        # 迭代求优
        temp_cur = np.zeros((3, self.Pt_num))
        delta_RMSE, last_RMSE = 1.0, 10086.0
        iter_cur = 0
        while delta_RMSE > 0.0001 and iter_cur < 5000:
            # 梯度计算
            grad_P = _calGradP()
            # 参数更新
            for m in range(3 * self.Pt_num - 6):
                Params[m] += 0.01 * grad_P[0][m]
            for n in range(self.Fig_N):
                Params[6 * n + 3 * self.Pt_num - 6:6 * n + 3 * self.Pt_num - 3] += 0.00005 * grad_P[0,
                                                                                             6 * n + 3 * self.Pt_num - 6:6 * n + 3 * self.Pt_num - 3]
                Params[6 * n + 3 * self.Pt_num - 3:6 * n + 3 * self.Pt_num] += 0.01 * grad_P[0,
                                                                                      6 * n + 3 * self.Pt_num - 3:6 * n + 3 * self.Pt_num]
            # 构造模板坐标temp_cur
            temp_cur[0][1] = Params[0]
            temp_cur[0][2] = Params[1]
            temp_cur[1][2] = Params[2]
            for i in range(3, 3 * self.Pt_num - 6):
                temp_cur[i % 3][i // 3 + 2] = Params[i]
            rmse_list = []
            # 计算误差
            for k in range(self.Fig_N):
                R_ = eulerAnglesToRotationMatrix(Params[6 * k + 3 * self.Pt_num - 6],
                                                 Params[6 * k + 3 * self.Pt_num - 5],
                                                 Params[6 * k + 3 * self.Pt_num - 4])
                T_ = Params[6 * k + 3 * self.Pt_num - 3:6 * k + 3 * self.Pt_num].reshape(-1, 1)
                loss_ = np.dot(R_, temp_cur) + T_ - measure3d[k]
                rmse_ = np.linalg.norm(loss_) / math.sqrt(self.Pt_num * 3)
                rmse_list.append(rmse_)
            rmse_ave = sum(rmse_list) / len(rmse_list)
            delta_RMSE = abs(rmse_ave - last_RMSE)
            last_RMSE = rmse_ave
            print("iter {}, rmse {:5f}".format(iter_cur, rmse_ave))
            iter_cur += 1
            # print("rmse list=\n", rmse_list)
        if last_RMSE < 0.1:
            if temp_cur[0][1] < 0:
                R = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
                self.Pt_temp = R @ temp_cur
            else:
                # 保存优化结果
                self.Pt_temp = temp_cur
            print("optimized result has been saved! current template is \n", self.Pt_temp)


if __name__ == '__main__':

    # 加载.mat数据
    measure_data = io.loadmat('Ps_50_for_temp_opt.mat')['Ps50']  # (MxN)x3
    template_data = measure_data[:4, :]
    temp = template()  # 实例化模板
    temp.setInitTemp(template_data)  # 设置初始模板，不存在就自动创建
    M = max(template_data.shape)
    measure_data3d = np.zeros((measure_data.shape[0] // M, 3, M))
    for i in range(measure_data3d.shape[0]):
        measure_data3d[i] = measure_data[M * i:M * (i + 1)].T
    temp.optTemp(measure3d=measure_data3d)
