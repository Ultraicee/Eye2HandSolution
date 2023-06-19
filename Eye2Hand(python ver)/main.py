import socket
import json
import time
import scipy.io as io

from Utils.utility import *

# 全局变量
T_B2C = loadFromYaml('T_B2C_opt.yaml')
temp_opt = loadFromYaml('temp_opt.yaml')


def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return True, sock
    except Exception:
        sock.close()
        return False, sock


def sendCMD(sock, cmd, params=None, id=1):
    if not params:
        params = []
    else:
        params = json.dumps(params)
    sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(cmd, params, id) + "\n"
    try:
        sock.sendall(bytes(sendStr, "utf-8"))
        ret = sock.recv(1024)
        jdata = json.loads(str(ret, "utf-8"))
        if "result" in jdata.keys():
            return True, json.loads(jdata["result"]), jdata["id"]
        elif "error" in jdata.keys():
            return False, json.loads(jdata["error"]), jdata["id"]
        else:
            return False, None, None
    except Exception:
        return False, None, None


def prepare_work(sock):
    # 设置伺服状态
    suc, servo_state, id = sendCMD(sock, "getServoStatus")  # 启用 1   未启用 0
    if servo_state == 0:
        sendCMD(sock, "setServoStatus", {"status": 1})
        time.sleep(0.1)

    # 获取机器人状态     停止0，暂停1，急停2，运行3， 错误4
    suc, result, id = sendCMD(sock, "getRobotState")
    if result == 4:
        # 清除报警状态
        sendCMD(sock, "clearAlarm", {"force": 0})
        time.sleep(0.3)
    elif result == 3:
        sendCMD(sock, "stop")

    # 获取电机状态并同步
    suc, result, id = sendCMD(sock, "getMotorStatus")
    if (result == 0):
        sendCMD(sock, "syncMotorStatus")
        time.sleep(0.3)


def cal_T_E2o(Ps_cam, arm_pose):
    # 计算T_E2o
    T_B2C = loadFromYaml('T_B2C_opt.yaml')
    temp_opt = loadFromYaml('temp_opt.yaml')
    [r, t] = kabsch(Ps_cam.T, temp_opt.T)
    T_C2o = Rt2T(r, t)
    T_E2B = pose2T(arm_pose)
    T_E2o = T_C2o @ (T_B2C @ T_E2B)
    return T_E2o


def motion_control_direct(ps_target, T_E2o_opt=None):
    """
    一步到位
    :param ps_target:
    :param T_E2o_opt:
    :return:
    """
    if T_E2o_opt is not None:
        T_E2o = T_E2o_opt
    else:
        T_E2o = loadFromYaml('T_E2o_opt.yaml')
    r, t = kabsch(temp_opt.T, ps_target.T)
    T_o2C_target = Rt2T(r, t)
    T_E2B_target = np.linalg.inv(T_B2C) @ (T_o2C_target @ T_E2o)  # T_C2B*T_o2C*T_E2o=T_E2B
    return T2pose(T_E2B_target)


def motion_control_iter(ps_target, ps_cur, arm_pose_cur):
    """
    差量变换，当前姿态变换到目标姿态
    :param ps_target:
    :param ps_cur:
    :param arm_pose_cur:
    :return:
    """
    r, t = kabsch(ps_cur.T, ps_target.T)
    T_C2Ci = Rt2T(r, t)
    T_E2B = pose2T(arm_pose_cur)
    T_E2B_target = np.linalg.inv(T_B2C)@T_C2Ci@T_B2C@T_E2B

    return T2pose(T_E2B_target)


def camera_msg(ip, port):
    sock_camera = socket.socket()
    sock_camera.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
    sock_camera.connect((ip, port))
    msg = sock_camera.recv(1024)
    sock_camera.close()
    return bytes2Mat(msg)


def bytes2Mat(data_bytes, idx=2):
    data_temp_cam = data_bytes.split(b'\n\n')[idx]
    data_p = data_temp_cam.split(b'\n')
    N = len(data_p)
    Ps = np.zeros((3, N))
    try:
        for i in range(N):
            data_axis = data_p[i].split(b',')
            Ps[0][i] = float(data_axis[0])
            Ps[1][i] = float(data_axis[1])
            Ps[2][i] = float(data_axis[2])
    except ValueError as e:
        print(e)
    return Ps


def move(sock_elite, arm_pose_target):
    suc, ik_move_result, id = sendCMD(sock_elite, "inverseKinematic", {"targetPose": arm_pose_target})
    assert ik_move_result is not None
    # print("ik_target_result:", ik_target_result)
    sendCMD(sock_elite, "moveByJoint", {"targetPos": ik_move_result, "speed": 30})
    time.sleep(3)
    while True:
        # print("running...")
        suc, state, id = sendCMD(sock_elite, "getRobotState")
        assert state is not None
        if state == 0:
            break
    camera_msg(camera_ip, camera_port)  # 清空缓存
    time.sleep(1)
    ps_end = camera_msg(camera_ip, camera_port)
    return ps_end


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
