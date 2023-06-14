## Eye2HandSolution

### 1. 描述

定义坐标系：o-模板（目标）坐标系、B-基坐标系、E-末端坐标系、C-相机坐标系；

具体算法推导见`手眼标定方程详解.pdf`

整个流程包含：机械臂、红外相机采集数据[`Ps_cam`, `pose_arm`]->算法求优->手眼协同；

### 2. 实现

提供了两个版本的实现：

- matlab：完成了`T_B2C`、`T_E2o`的求解
- python：待完成

matlab版本使用fmincon求优工具箱实现了求优，python使用SciPy求优库的minimize函数求优（目前效果不佳，待改进）

### 3. TODO

- 完成python版本的求优环节；

- 完成后续测试：在相机坐标系下设定一个目标位姿T_{o2C}^{target}，可基于模板在相机坐标系下坐标P_cam，使用Kabsch计算T_{o2C}^{target}；

  根据机械臂当前位姿pose_arm_cur推算T_{E2B}^{target}，下发给机械臂，T_{E2B}^{target}=**T_C2B***T_{o2C}^{target}***T_E2o**; 

  T_{E2B}^{cur} + T_{E2B}^{motion} -> T_{E2B}^{target} (可能非线性相加)

  结果应该是向目标位姿靠近，连续迭代几次应该越来越靠近；
