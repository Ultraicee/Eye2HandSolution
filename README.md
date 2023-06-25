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

### 3. TODO

- 完成python版本的求优环节；

- 优化手眼协同；

- test
