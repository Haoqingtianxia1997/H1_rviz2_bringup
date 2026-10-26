"""
计算 zed_left_camera_optical_frame 相对于 imu 的外参
并在 Open3D 中可视化三个坐标系：imu / zed_left_camera_optical_frame / torso_joint

运动链（均以 torso_link 为公共参考系）：
  torso_link
    ├── imu_joint              → imu_link
    ├── torso_joint (origin)   → torso_link 原点
    └── camera_mount1_joint
          └── camera_support2_joint
                └── camera_frame joint
                      └── zed_left_camera_optical_frame joint
                            → zed_left_camera_optical_frame
"""

import os
import yaml
import numpy as np
import open3d as o3d


# ─────────────────────────────────────────────
# 工具函数
# ─────────────────────────────────────────────

def rpy_to_R(roll, pitch, yaw):
    """URDF RPY → 旋转矩阵（外旋 XYZ = R_z @ R_y @ R_x）"""
    Rx = np.array([[1,             0,              0],
                   [0,  np.cos(roll), -np.sin(roll)],
                   [0,  np.sin(roll),  np.cos(roll)]])
    Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                   [             0, 1,             0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw),  np.cos(yaw), 0],
                   [          0,            0, 1]])
    return Rz @ Ry @ Rx


def make_T(xyz, rpy):
    """构造 4×4 齐次变换矩阵"""
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3]  = xyz
    return T


# ─────────────────────────────────────────────
# 从 YAML 加载各关节变换参数（torso_link 为参考系）
# ─────────────────────────────────────────────
_cfg_path = os.path.join(os.path.dirname(__file__), "extrinsic_config.yaml")
with open(_cfg_path, "r") as _f:
    _cfg = yaml.safe_load(_f)["transforms"]

def _T(key): return make_T(_cfg[key]["xyz"], _cfg[key]["rpy"])

# torso_joint: 参考原点
T_torso_joint = make_T([0, 0, 0], [0, 0, 0])

# imu_joint: parent=torso_link
T_imu_in_torso          = _T("imu_in_torso")

# camera_mount1_joint: parent=torso_link
T_mount1_in_torso       = _T("camera_mount1_in_torso")

# camera_support2_joint: parent=camera_mount1
T_support2_in_mount1    = _T("camera_support2_in_mount1")

# camera_frame joint: parent=camera_support2
T_cam_frame_in_support2 = _T("camera_frame_in_support2")

# zed_left_camera_optical_frame: parent=camera_frame  (from tf2_echo)
T_optical_in_cam_frame  = _T("optical_in_camera_frame")

# ─────────────────────────────────────────────
# 计算 zed_left_camera_optical_frame 在 torso_link 中的变换
# ─────────────────────────────────────────────
T_cam_in_torso = T_mount1_in_torso @ T_support2_in_mount1 @ T_cam_frame_in_support2 @ T_optical_in_cam_frame

# ─────────────────────────────────────────────
# 外参：zed_left_camera_optical_frame 相对于 imu
#   T_cam_in_imu = T_imu_in_torso⁻¹ @ T_cam_in_torso
# ─────────────────────────────────────────────
T_cam_in_imu = np.linalg.inv(T_imu_in_torso) @ T_cam_in_torso

# ─────────────────────────────────────────────
# 打印结果
# ─────────────────────────────────────────────
np.set_printoptions(precision=6, suppress=True)

print("=" * 60)
print("各坐标系在 torso_link 中的位置")
print("=" * 60)
print("\n[torso_joint]  translation:", T_torso_joint[:3, 3])
print("[imu]          translation:", T_imu_in_torso[:3, 3])
print("[left_camera]  translation:", T_cam_in_torso[:3, 3])

print("\n" + "=" * 60)
print("外参：zed_left_camera_optical_frame 相对于 imu")
print("  T_cam_in_imu  =  T_imu_in_torso⁻¹  @  T_cam_in_torso")
print("=" * 60)
print("\n平移 (m):")
print(T_cam_in_imu[:3, 3])
print("\n旋转矩阵:")
print(T_cam_in_imu[:3, :3])

# 转换为四元数 (x, y, z, w)
R = T_cam_in_imu[:3, :3]
trace = R[0,0] + R[1,1] + R[2,2]
if trace > 0:
    s = 0.5 / np.sqrt(trace + 1.0)
    qw = 0.25 / s
    qx = (R[2,1] - R[1,2]) * s
    qy = (R[0,2] - R[2,0]) * s
    qz = (R[1,0] - R[0,1]) * s
else:
    if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        qw = (R[2,1] - R[1,2]) / s
        qx = 0.25 * s
        qy = (R[0,1] + R[1,0]) / s
        qz = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        qw = (R[0,2] - R[2,0]) / s
        qx = (R[0,1] + R[1,0]) / s
        qy = 0.25 * s
        qz = (R[1,2] + R[2,1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        qw = (R[1,0] - R[0,1]) / s
        qx = (R[0,2] + R[2,0]) / s
        qy = (R[1,2] + R[2,1]) / s
        qz = 0.25 * s

print(f"\n四元数 (x, y, z, w): [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]")
print("\n完整 4×4 矩阵:")
print(T_cam_in_imu)


# ─────────────────────────────────────────────
# Open3D 可视化
# ─────────────────────────────────────────────
FRAME_SIZE = 0.08   # 坐标轴长度 (m)

# 三个坐标系，均以 torso_link 为公共坐标系
frames_def = {
    "torso_joint (红=X 绿=Y 蓝=Z)":          (T_torso_joint,  FRAME_SIZE),
    "imu":                                    (T_imu_in_torso, FRAME_SIZE),
    "zed_left_camera_optical_frame":          (T_cam_in_torso, FRAME_SIZE),
}

geometries = []

for name, (T, size) in frames_def.items():
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    frame.transform(T)
    geometries.append(frame)

# 用线段连接三个坐标系原点，方便观察相对位置
pts = np.array([
    T_torso_joint[:3, 3],
    T_imu_in_torso[:3, 3],
    T_cam_in_torso[:3, 3],
])
lines  = [[0, 1], [0, 2], [1, 2]]
colors = [[0.5, 0.5, 0.5]] * 3

line_set = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector(pts),
    lines=o3d.utility.Vector2iVector(lines),
)
line_set.colors = o3d.utility.Vector3dVector(colors)
geometries.append(line_set)

# 添加一个小球标记 imu 位置
sphere_imu = o3d.geometry.TriangleMesh.create_sphere(radius=0.012)
sphere_imu.paint_uniform_color([1.0, 0.6, 0.0])   # 橙色 = imu
sphere_imu.translate(T_imu_in_torso[:3, 3])
geometries.append(sphere_imu)

# 添加一个小球标记 camera 位置
sphere_cam = o3d.geometry.TriangleMesh.create_sphere(radius=0.012)
sphere_cam.paint_uniform_color([0.0, 0.8, 1.0])   # 青色 = camera
sphere_cam.translate(T_cam_in_torso[:3, 3])
geometries.append(sphere_cam)

print("\n" + "=" * 60)
print("Open3D 窗口说明（公共坐标系 = torso_link）：")
print("  大坐标轴(原点) = torso_joint")
print("  橙色小球       = imu")
print("  青色小球       = zed_left_camera_optical_frame")
print("  坐标轴 红=X  绿=Y  蓝=Z")
print("=" * 60)

o3d.visualization.draw_geometries(
    geometries,
    window_name="Extrinsic: imu / zed_left_camera_optical_frame / torso_joint  (公共坐标系=torso_link)",
    width=1200,
    height=800,
    point_show_normal=False,
)
