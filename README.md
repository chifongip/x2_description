# Robot Model

## 启动 (Launch)

在 RViz 中查看机器人模型：

```bash
# 默认使用 x2_ultra.urdf
ros2 launch x2_description display.launch.py

# 指定其他模型
ros2 launch x2_description display.launch.py urdf:=x2_fist.urdf
ros2 launch x2_description display.launch.py urdf:=x2_hand.urdf
ros2 launch x2_description display.launch.py urdf:=x2_ultra_simple_collision.urdf
```

`urdf` 参数为 `urdf/` 目录下的文件名。

## Odin mounting calibration

`x2_ultra.urdf` includes this fixed sensor chain:

```text
torso_link -> odin_imu -> odin_lidar -> odin_camera_optical
```

The camera mounting pose uses checkerboard **result_005**, captured on 2026-10-20.
The `torso_to_odin_imu` joint uses its exported `T_torso_odin_imu` directly.
The solver derives this mounting from the calibrated camera pose:

```text
T_torso_imu = T_torso_camera_calibrated * Tcl_0 * inverse(T_imu_lidar)
```

The resulting `torso_to_odin_imu` translation is
**[0.073728756432, -0.039674780145, 0.519151522653] m**, with RPY
**[-0.017112521613, -0.116099874088, 0.029548196340] rad**.
`T_A_B` maps coordinates from B into A.

The internal sensor joints use factory geometry:

- `odin_imu_to_lidar`: documented fixed translation
  **[-0.02663, 0.03447, 0.02174] m**, with identity rotation.
- `odin_lidar_to_camera_optical`: inverse of device **O1-P070100425**'s
  `Tcl_0` from `odin_ros_driver/config/calib.yaml`. The decimal-rounded factory
  rotation is projected onto SO(3) using SVD before inversion and conversion to
  URDF RPY. Translation is retained as supplied.

These replace the previous averages of `/odin1/wiwc` matrices, which disagreed
with the documented hardware geometry. The composed torso-to-camera pose matches
session_005. The camera optical axes are x right, y down, z forward.
See the [manufacturer's extrinsic definitions](https://manifoldtechltd.github.io/wiki/en/odin_series/odin1/5.%20Data%20output_.html#52-extrinsic-description).

This calibration remains **provisional (`requires_review`)**: Odin validation
RMS was 4.82 px (p95 10.26 px), and mounting repeatability did not meet the
configured targets. The solve retained 26 of 30 training pairs and used 10
validation pairs at one head posture.
It is intended for visualization and integration checks pending validation.
No Odin collision geometry, meshes, inertial properties, or joint limits are
introduced by these frame-only links.

The prefixed frame names deliberately differ from the driver's native `imu`,
`lidar`, and `camera_0`. Do not add identity aliases between the two trees while
the driver owns `odom -> imu` and `odom -> camera_0`: that would give frames
multiple parents. A later localization adapter must explicitly coordinate TF
ownership and convert native sensor odometry into the robot's base frame.
This URDF change does not replace FAST-LIO or reconfigure the Odin driver.

Shared `x2_bringup` includes this URDF, so it receives the new fixed frames after
its existing state publisher reloads/restarts. Do not start a second state
publisher or controller manager. The alternative robot models are unchanged.

### 可用模型

| 文件 | 说明 |
| --- | --- |
| `x2_ultra.urdf` | 默认模型 |
| `x2_ultra_simple_collision.urdf` | 简化碰撞体 |
| `x2_hand.urdf` | 手爪末端 |
| `x2_fist.urdf` | 拳形末端 |
| `*.xml` / `scene.xml` | MJCF (MuJoCo) 格式 |

## DEBUG工具
### xml && urdf
```
python -m mujoco.viewer
```
* 需要安装mujoco
```
pip install mujoco
```
### urdf
* VScode插件
``` 
# URDF Visualizer
```
## x2_ultra (plus)
![alt text](visual/x2_ultra.png)
## x2_hand (plus)
![alt text](visual/x2_hand.png)
## x2_fist (plus)
![alt text](visual/x2_fist.png)
