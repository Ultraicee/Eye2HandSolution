function T_e2b = pose2T_E2B(Pose)
% pose2RT 转换机械臂位姿为旋转平移矩阵
% Pose 1x6 分别为 [Px Py Pz  Rx, Ry, Rz]
% R    3x3 基坐标系到末端坐标系（R_b2e）
% t    3x1 基坐标系到末端坐标系（t_b2e）
    Px = Pose(1);
    Py = Pose(2);
    Pz = Pose(3);
    R_e2b = angle2dcm(Pose(6), Pose(5), Pose(4));
    t_e2b= [Px Py Pz]';  % 末端位姿相对基坐标原点的平移
    T_e2b = [R_e2b', t_e2b;0 0 0 1]; % 被动旋转需要转置，由坐标轴旋转变成点旋转
end

