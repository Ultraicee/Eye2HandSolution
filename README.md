## Eye2HandSolution

### 1. 描述

定义坐标系：o-模板（目标）坐标系、B-基坐标系、E-末端坐标系、C-相机坐标系；

具体算法推导见`手眼标定方程详解.pdf`

整个流程包含：机械臂、红外相机采集数据[`Ps_cam`, `pose_arm`]->算法求优->手眼协同；

手眼协同测试：设定目标位置（Ps_{target}），计算出目标位置对应的arm pose，控制机械臂完成运动，计算终止位置Ps_{end}与目标位置的距离误差RMSE。

### 2. 实现

提供了两个版本的实现：

- matlab：完成了`T_B2C`、`T_E2o`的求解
- python：待完成优化环节

matlab版本使用fmincon求优工具箱实现了`T_B2C`的求优，python使用SciPy求优库的minimize函数求优（目前效果不佳，待改进）

python版本的`main.py`完成了：
- 接收相机定位数据
- 可选自动更新`T_E2o`
- 计算设定目标位置所需的机械臂参数arm pose=[Px,Py,Pz,Rx,Ry,Rz]
- 控制机械臂运动并计算运动误差rmse(target, end)

### 3. Usage

S1: 使用`Eye2Hand(python ver)/auto_collect_proc.py`完成数据采集（单轴移动和自由采集N组），实际上单轴移动采集的数据也可用于求优`T_B2C`，但相同数据既提供初值又完成求优效果会差一些；

S2: 将导出的`Pc_single_axis.mat`和`P_C_N_and_arm_pose_N.mat`导入MATLAB，在`Eye2Hand(matlab ver)/calRT_CAM2BASE_v2.m`求解`T_B2C`；

S3: 手动更新`T_B2C_opt.yaml`文件；
### 4. TODO

- 完成python版本的求优环节，避免用MATLAB折中处理；

- 优化手眼协同；

### 5. Note
在求解t_B2C时，使用被动标识球靠近机械臂底座，此时读取到的相机坐标系下坐标值接近t_B2C!!!
