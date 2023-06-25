import socket
import json
import time

from Utils.utility import *

camera_ip = "192.168.2.83"  # 相机端IP地址
# camera_ip = "127.0.0.1"  # 相机端IP地址
robot_ip = "192.168.1.200"  # 机械臂IP地址（端口只能选择8055）


def connectETController(ip, port=8055):
    """
    连接电机控制器
    Args:
        ip: 机械臂IP地址
        port: 端口号

    Returns:

    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return True, sock
    except ConnectionError as e:
        print(e)
        sock.close()
        return False, sock


def sendCMD(sock, cmd, params=None, id=1):
    """
    发送电机控制指令
    Args:
        sock:
        cmd: 指令
        params:
        id:

    Returns:

    """
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
    except Exception as e:
        print(e)
        return False, None, None


def prepare_work(sock):
    """
    电机状态初始化
    Args:
        sock:

    Returns:

    """
    # 设置伺服状态
    suc, servo_state, id = sendCMD(sock, "getServoStatus")  # 启用1，未启用0
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


def camera_msg(ip, port=8765):
    """
    捕获双目相机传输的点数据
    Args:
        ip: 相机IP地址
        port: 相机端口号

    Returns:
        输出点数据
    """
    sock_camera = socket.socket()
    sock_camera.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
    sock_camera.connect((ip, port))
    msg = sock_camera.recv(1024)
    sock_camera.close()
    return bytes2Mat(msg)


def bytes2Mat(data_bytes, idx=2):
    """
    根据PS_For_IR的传输方式(特殊字符分割，第3个为检测空间点数据)特定写法
    Args:
        data_bytes: bytes, TCP传输的数据
        idx: int, 数据在结构体中索引

    Returns: ndarray, 点数据

    """
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


def single_axis_move(sock_elite, bias=40):
    """
    单轴移动
    Args:
        sock_elite:
        bias: 单轴运动距离
    Returns:
        保存Pc_single_axis.mat文件，保存标识球在相机坐标系下数据
    """
    suc, arm_pose_origin, id = sendCMD(sock_elite, "getRobotPose")
    print("initial arm pose:", arm_pose_origin)
    P_C_single_axis = np.zeros((4 * 6, 3))
    for i in range(3):
        arm_pose_origin[i] -= bias
        P_C_single_axis[8 * i:8 * i + 4, :] = move(sock_elite, arm_pose_origin).T
        arm_pose_origin[i] += 2 * bias
        P_C_single_axis[8 * i + 4:8 * (i + 1), :] = move(sock_elite, arm_pose_origin).T
        arm_pose_origin[i] -= bias  # 还原
    print("data preview:\n", P_C_single_axis)
    io.savemat('Pc_single_axis.mat', {'Pc_single_axis': P_C_single_axis})


def collet_N_move(sock_elite):
    """
    收集N组自由移动数据
    Args:
        sock_elite:

    Returns:

    """
    P_C_N = np.zeros((4 * 5, 3))
    arm_pose_N = np.zeros((5, 6))
    for i in range(5):
        input("[{}/5] collect current pose?".format(i + 1))
        suc, arm_pose_end, id = sendCMD(sock_elite, "getRobotPose")
        print("arm pose=", arm_pose_end)
        camera_msg(camera_ip)  # 清空缓存
        time.sleep(2)
        while True:
            ps_end = camera_msg(camera_ip)
            print("current position(May not Sync!!!check with PS_for_IR):\n", ps_end.T)
            message = input(
                "Check position[Y] Other Key to refresh it!\n")
            if message == 'Y' or message == 'y':
                P_C_N[4 * i:4 * (i + 1), :] = camera_msg(camera_ip).T
                break
        arm_pose_N[i, :] = np.array(arm_pose_end)
    print("data preview:\n", P_C_N)
    io.savemat('P_C_N_and_arm_pose_N.mat', {'P_C_N': P_C_N, 'arm_pose_N': arm_pose_N})


def move(sock_elite, arm_pose_dst):
    """
    控制机械臂运动到目标位姿
    Args:
        sock_elite:
        arm_pose_dst:机械臂目标姿态

    Returns:

    """
    # X axis Add
    suc, ik_move_result, id = sendCMD(sock_elite, "inverseKinematic", {"targetPose": arm_pose_dst})
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
    camera_msg(camera_ip)  # 清空缓存
    time.sleep(1)
    suc, arm_pose_end, id = sendCMD(sock_elite, "getRobotPose")
    if np.linalg.norm(np.array(arm_pose_end) - np.array(arm_pose_dst)) < 10:
        ps_end = camera_msg(camera_ip)
        return ps_end
    else:
        print("error!move failure!")
        return


if __name__ == "__main__":
    # 与机械臂建立连接
    conSuc, sock_elite = connectETController(robot_ip, port=8055)
    print("connect status:", conSuc)
    prepare_work(sock_elite)
    # 反注释指定模块启用功能
    """ 机械臂单轴移动并采集数据 """
    # single_axis_move(sock_elite)

    """ 手动控制机械臂位姿并确认收集 """
    # collet_N_move(sock_elite)

    print("Done!")
