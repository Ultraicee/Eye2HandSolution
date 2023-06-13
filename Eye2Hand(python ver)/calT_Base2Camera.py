import numpy as np
from Utils.utility import *
from scipy.optimize import minimize
from build_template import template


def calR(Ps_cam):
    """
    description:
        calR calculate initial rotation matrix between BASE coordinate system and CAMERA
    :param Ps_cam: (3x6) x M, 6组标识球数据
    :return:
    """
    # x axis motion
    r_1 = cal_rot_vec(Ps_cam[:4, :].T, Ps_cam[4:8, :].T)
    # y axis motion
    r_2 = cal_rot_vec(Ps_cam[8:12, :].T, Ps_cam[12:16, :].T)
    # z axis motion
    r_3 = cal_rot_vec(Ps_cam[16:20, :].T, Ps_cam[20:24, :].T)
    R_B2C_init, _ = kabsch(np.eye(3), np.array([r_1, r_2, r_3]))
    return R_B2C_init


def cost_func(para, A, B):
    R_B2C = angle2dcm(para[:3])
    t_B2C = para[3:].reshape(3, 1)
    T_B2C = Rt2T(R_B2C, t_B2C)
    N = A.shape[-1]
    delta_4N = np.zeros((4 * (N - 1), 4))
    for i in range(1, N):
        delta_4N[(i - 1) * 4: i * 4, :] = T_B2C @ A[:, :, i] - B[:, :, i] @ T_B2C
    err = np.linalg.norm(delta_4N) / math.sqrt(4 * N)
    return err


if __name__ == '__main__':
    # 加载.mat数据
    data_dict = data_load(mat_file_path='measure_camPos4x3_armPos1x6.mat',
                          keyword_list=['P_C', 'P_C_N', 'pose_arm_N'])
    temp_opt = np.array([[0.00000000e+00, 9.68285131e+01, 5.56483956e+01, -4.82045988e+01],
                         [0.00000000e+00, 0.00000000e+00, 4.20890318e+01, -1.34696577e+01],
                         [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -1.26345480e-02]])
    t_B2C_init = np.array([-188.510, 82.918, 790.387])  # 使用标识球靠近底座位置在光学定位系统上粗略定位
    ps_single_axis = data_dict['P_C']  # 用于单轴移动的数据
    ps_opt = data_dict['P_C_N']  # 用于求优T_B2C的标识球数据
    pose_arm = data_dict['pose_arm_N']  # 用于求优T_B2C的机械臂位姿数据
    template_data = ps_opt[:4, :]  # 取第一组数据初始化模板
    # if temp_opt is None:
    #     temp_opt = template()  # 实例化模板
    #     temp_opt.setInitTemp(template_data)  # 设置初始模板，不存在就自动创建
    #     M = max(template_data.shape)
    #     measure_data3d = np.zeros((ps_opt.shape[0] // M, 3, M))
    #     for i in range(measure_data3d.shape[0]):
    #         measure_data3d[i] = ps_opt[M * i:M * (i + 1)].T
    #     temp_opt.optTemp(measure3d=measure_data3d)
    #     print(temp_opt.getTemp())

    # 求解R_B2C_init
    R_B2C_init = calR(ps_single_axis[4:, :])
    # 构造A和B
    N = pose_arm.shape[0]
    T_C2C = np.zeros((4, 4, N))
    A_N = np.zeros((4, 4, N))
    T_E2B1 = pose2T(pose_arm[0, :])
    for i in range(N):
        R_C2C_i, t_C2C_i = kabsch(ps_opt[0:4, :], ps_opt[i * 4:(i + 1) * 4, :])
        T_C2C[:, :, i] = Rt2T(R_C2C_i, t_C2C_i)
        T_E2Bi = pose2T(pose_arm[i])
        # 计算R_B2C_init的err
        A_N[:, :, i] = T_E2Bi @ np.linalg.inv(T_E2B1)
        R_B = T_C2C[:3, :3, i]
        R_A = A_N[:3, :3, i]
        t_A = A_N[:3, 3, i]
        t_B = T_C2C[:3, 3, i]
        delta_err = R_B2C_init @ R_A - R_B @ R_B2C_init
        err = np.linalg.norm(delta_err) / math.sqrt(3 * 3)
    B_N = T_C2C
    # TODO:完成优化过程,目前rmse=6.132，MATLAB可达到0.31
    # 求优T_B2C
    angles = dcm2angle(R_B2C_init)
    para0 = np.hstack((angles, t_B2C_init))
    res = minimize(cost_func, para0, method='SLSQP', args=(A_N, B_N),
                 options={'maxiter': 3000, 'eps': 0.001, 'ftol': 0.0001, 'disp': True})
    T = Rt2T(angle2dcm(res.x[:3]), res.x[3:])
    print(T)

    T_B2C = np.array([[0.4857, 0.8731, -0.0418, -226.1417],
                      [0.2429, -0.1807, -0.9531, 120.1081],
                      [-0.8397, 0.4528, -0.2999, 906.8772],
                      [0, 0, 0, 1.0000]])  # MATLAB求解结果
    # 计算T_E2o
    [r, t] = kabsch(ps_opt[:4, :], temp_opt.T)
    T_C2o = Rt2T(r, t)
    T_E2B = pose2T(pose_arm[0, :])
    T_E2o = T_C2o @ (T_B2C @ T_E2B)
