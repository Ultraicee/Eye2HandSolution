import socket
import time

from Utils.utility import *
from auto_collect_proc import connectETController, sendCMD, prepare_work, camera_msg, move
# 全局变量
T_B2C = loadFromYaml('T_B2C_opt.yaml')
temp_opt = loadFromYaml('temp_opt.yaml')


def cal_T_E2o(Ps_cam, arm_pose):
    """
    基于T_B2C计算机械臂末端到模板（目标）的变换矩阵T_E2o
    Args:
        Ps_cam: 采集到的标识球坐标数据
        arm_pose: 机械臂末端位姿(Elite机械臂读取)

    Returns:

    """
    # 计算T_C2o
    [r, t] = kabsch(Ps_cam.T, temp_opt.T)
    T_C2o = Rt2T(r, t)
    T_E2B = pose2T(arm_pose)
    T_E2o = T_C2o @ (T_B2C @ T_E2B)  # E->B->C->o
    return T_E2o


def motion_control_direct(ps_dst, T_E2o_opt=None):
    """
    计算目标位置的T_E2B_target完成运动
    :param ps_dst: 设定模板（目标）在相机坐标系下目标坐标值
    :param T_E2o_opt: 优化过的末端到模板（目标）的变换矩阵T
    :return: 机械臂末端所需的6参数（Px, Py, Pz, Rx, Ry, Rz）
    """
    if T_E2o_opt is not None:
        T_E2o = T_E2o_opt
    else:
        T_E2o = loadFromYaml('T_E2o_opt.yaml')
    r, t = kabsch(temp_opt.T, ps_dst.T)
    T_o2C_dst = Rt2T(r, t)
    T_E2B_dst = np.linalg.inv(T_B2C) @ (T_o2C_dst @ T_E2o)  # T_C2B*T_o2C*T_E2o=T_E2B
    return T2pose(T_E2B_dst)


def motion_control_iter(ps_dst, ps_cur, arm_pose_cur):
    """
    差量控制，基于当前姿态变换到目标姿态
    :param ps_dst: 设定模板（目标）在相机坐标系下目标坐标值
    :param ps_cur: 模板（目标）在相机坐标系下当前坐标值
    :param arm_pose_cur: 机械臂当前位姿参数
    :return: 机械臂末端所需的6参数（Px, Py, Pz, Rx, Ry, Rz）
    """
    r, t = kabsch(ps_cur.T, ps_dst.T)  # T_C2C_{dst}
    T_C2Ci = Rt2T(r, t)
    T_E2B = pose2T(arm_pose_cur)
    T_E2B_dst = np.linalg.inv(T_B2C)@T_C2Ci@T_B2C@T_E2B  # E->B->C->C_{dst}->B（应该存在问题）
    return T2pose(T_E2B_dst)


if __name__ == "__main__":
    robot_ip = "192.168.1.200"  # 机械臂IP地址（端口只能选择8055）
    # 与机械臂建立连接
    conSuc, sock_elite = connectETController(robot_ip, port=8055)
    print("connect status:", conSuc)
    prepare_work(sock_elite)
    suc, arm_pose_temp, id = sendCMD(sock_elite, "getRobotPose")
    print("initial arm pose:", arm_pose_temp)

    camera_ip = "192.168.2.11"  # 相机端IP地址
    # camera_ip = "127.0.0.1"  # 相机端IP地址
    camera_port = 8765
    T_E2o_opt = None
    Ps_target = io.loadmat('P_C_N_and_arm_pose_N.mat')['P_C_N'].T  # 3x(N*M)

    N = Ps_target.shape[1] // 4
    for i in range(N):
        print("======================Target Motion Test({}/{})==========================".format(i + 1, N))
        ps_target = Ps_target[:, i * 4:(i + 1) * 4]
        # 仅首次询问
        while i < 1:
            ps_temp = camera_msg(camera_ip, camera_port)
            print("current position(May not Sync!!!check with PS_for_IR):\n", ps_temp.T)
            suc, arm_pose_temp, id = sendCMD(sock_elite, "getRobotPose")
            print("respond arm Pos:", arm_pose_temp)
            message = input("[1/3] Update T_E2o?[Y/n]\n")

            if message == 'Y' or message == 'y':
                T_E2o_opt = cal_T_E2o(ps_temp, arm_pose_temp)
                saveToYaml(T_E2o_opt, 'T_E2o_opt.yaml')
                print("T_E2o_opt=\n", T_E2o_opt)
                break
            elif message == 'N' or message == 'n':
                break
        arm_pose_target = motion_control_direct(ps_target, T_E2o_opt)
        ps_end = move(sock_elite, arm_pose_target.tolist())
        rmse = np.linalg.norm(ps_end - ps_target) / math.sqrt(ps_target.size)
        print("[direct]rmse(target, end)={:.5f}".format(rmse))
        # max_iter_times = 5
        # iter_times = 0
        # # 迭代靠近目标点
        # while rmse > 5 and iter_times < max_iter_times:
        #     camera_msg(camera_ip, camera_port)  # 清空缓存
        #     suc, arm_pose_cur, id = sendCMD(sock_elite, "getRobotPose")
        #     assert arm_pose_cur is not None
        #     # print("current arm_pose:", arm_pose_cur)
        #     suc, ik_cur_result, id = sendCMD(sock_elite, "inverseKinematic", {"targetPose": arm_pose_cur})
        #     assert ik_cur_result is not None
        #     time.sleep(2)
        #     ps_cur = camera_msg(camera_ip, camera_port)
        #     arm_pose_target = motion_control_iter(ps_target, ps_cur, arm_pose_cur)  # 未使用T_E2o
        #     arm_pose_target = arm_pose_target.tolist()
        #     print("target arm_pose:", arm_pose_target)
        #     # print("target position:\n", ps_target.T)
        #     suc, ik_target_result, id = sendCMD(sock_elite, "inverseKinematic", {"targetPose": arm_pose_target})
        #     if ik_target_result is not None:
        #         print("ik_diff(current->target): ", np.array(ik_cur_result) - np.array(ik_target_result))
        #         # message1 = input("[2/3] MOVE Arm to Target Pos?[Y/n]\n")
        #         # if message1 == 'Y' or message1 == 'y':
        #         ps_end = move(sock_elite, arm_pose_target).T
        #         # 捕获到正确位置数据后，计算误差值
        #         rmse = np.linalg.norm(ps_end - ps_target) / math.sqrt(ps_target.size)
        #         print("[{}/{}]rmse(target, end)={:.5f}".format(iter_times + 1, max_iter_times, rmse))
        #         iter_times += 1

    print("Done!")
