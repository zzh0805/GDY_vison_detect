# -*- coding: utf-8 -*-
"""手眼标定数值求解（眼在手外 Eye-to-Hand，纯 numpy/scipy，不依赖 OpenCV 版本）。

约定（与现场一致）：
    X = T_base_camera   待求手眼矩阵，P_base = X @ P_camera
    Y = T_flange_board  标定板相对法兰的固定位姿（常数）
    约束： X @ T_cam_board_i = T_base_flange_i @ Y    (i = 1..N)

提供:
    solve_shah_robot_world(A, B)   # 线性初解 AX=YB（Shah 2013，Kronecker 积 + SVD）
    solve_park_axxb(A, B)          # 交叉验证 AX=XB（Park & Martin，角轴 Procrustes）
    make_axxb_pairs(A, B)          # 由 A/B 生成 AX=XB 位姿对
    refine_ax_yb(A, B, X0, Y0)     # 非线性联合精化（scipy least_squares，旋转+平移残差）
    verify_ax_yb(A, B, X, Y)       # 残差 / Y 一致性 / 姿态分散度验证
    selftest()                     # 合成数据自检：验证能精确恢复 X

输入输出统一用 4x4 齐次矩阵 (N,4,4)；平移单位：米（与 UR 一致）。
"""
from __future__ import annotations

from typing import Iterable, List, Optional, Tuple

import numpy as np


# ============================================================
# 旋转 / 位姿 基本工具
# ============================================================
def _rotvec_to_mat(r: np.ndarray) -> np.ndarray:
    """Rodrigues：旋转向量 -> 旋转矩阵（r 单位 rad）。"""
    r = np.asarray(r, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.eye(3)
    k = r / angle
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


def _mat_to_rotvec(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> 旋转向量（rad），对角接近 pi 的情形做了稳健处理。"""
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt
    cos_a = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    a = float(np.arccos(cos_a))
    if a < 1e-8:
        return np.zeros(3)
    if np.pi - a < 1e-6:  # 接近 pi：用对称部分求轴
        w = (R + np.eye(3)) / 2.0
        evals, evecs = np.linalg.eigh(w)
        axis = evecs[:, int(np.argmax(evals))]
        return axis * a
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2.0 * np.sin(a)) * a


def homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = np.asarray(R, dtype=np.float64).reshape(3, 3)
    M[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return M


def compose(*ms: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    for m in ms:
        out = out @ m
    return out


def angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """两个旋转矩阵的夹角（度）。"""
    d = float(np.trace(Ra @ Rb.T) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(d, -1.0, 1.0))))


def rotation_axis_angle(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """返回 (轴, 角度度)。"""
    r = _mat_to_rotvec(R)
    a = float(np.linalg.norm(r))
    if a < 1e-10:
        return np.array([1.0, 0.0, 0.0]), 0.0
    return r / a, np.degrees(a)


def mean_rotation(Rs: Iterable[np.ndarray]) -> np.ndarray:
    """一组旋转矩阵的平均（SVD 投影到 SO(3)）。"""
    acc = np.zeros((3, 3))
    for R in Rs:
        acc += np.asarray(R, dtype=np.float64).reshape(3, 3)
    U, _, Vt = np.linalg.svd(acc)
    Rm = U @ Vt
    if np.linalg.det(Rm) < 0:
        U[:, -1] *= -1.0
        Rm = U @ Vt
    return Rm


def _project_rotation(M: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1.0
        R = U @ Vt
    return R


def _rotation_from_null_block(block: np.ndarray) -> np.ndarray:
    """从 Shah 零空间 9 维块提取旋转。

    零空间向量可能带任意（可正可负）的缩放系数：block ≈ c·R。
    先归一化尺度到 √3（≈ ±旋转矩阵），负号整块取反（对应 c<0），
    再做 SVD 正交化。避免奇异值退化时单列翻转无法恢复正确符号的问题。
    """
    M0 = np.asarray(block, dtype=np.float64).reshape(3, 3, order="F")
    n = float(np.linalg.norm(M0))
    if n < 1e-12:
        return np.eye(3)
    M0 = M0 / n * np.sqrt(3.0)
    if np.linalg.det(M0) < 0:
        M0 = -M0
    return _project_rotation(M0)


# ============================================================
# 1) Shah 线性法：直接解 AX=YB（机器人-世界/手眼联合标定）
#    X B_i = A_i Y
# ============================================================
def solve_shah_robot_world(A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Shah 2013 线性解。

    A: (N,4,4) T_base_flange；B: (N,4,4) T_cam_board。
    返回 X=T_base_cam, Y=T_flange_board（4x4）。
    """
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    N = len(A)
    if N < 3:
        raise ValueError(f"至少需要 3 个位姿，当前 {N} 个")

    # ---- 旋转： (RB^T ⊗ I) vec(RX) - (I ⊗ RA) vec(RY) = 0 ----
    # 注意：Kronecker 恒等式采用列主序 vec（vec(AXB)=(B^T⊗A)vec(X)）。
    M = np.zeros((9 * N, 18))
    for i in range(N):
        RA = A[i][:3, :3]
        RB = B[i][:3, :3]
        M[9 * i:9 * i + 9, 0:9] = np.kron(RB.T, np.eye(3))
        M[9 * i:9 * i + 9, 9:18] = -np.kron(np.eye(3), RA)
    _, _, vt = np.linalg.svd(M)
    v = vt[-1]  # 最小奇异值对应的零空间方向
    RX = _rotation_from_null_block(v[:9])
    RY = _rotation_from_null_block(v[9:18])

    # ---- 平移： tX - RA tY = tA - RX tB ----
    P = np.zeros((3 * N, 6))
    q = np.zeros(3 * N)
    for i in range(N):
        P[3 * i:3 * i + 3, 0:3] = np.eye(3)
        P[3 * i:3 * i + 3, 3:6] = -A[i][:3, :3]
        q[3 * i:3 * i + 3] = A[i][:3, 3] - RX @ B[i][:3, 3]
    sol, *_ = np.linalg.lstsq(P, q, rcond=None)
    X = homogeneous(RX, sol[0:3])
    Y = homogeneous(RY, sol[3:6])
    return X, Y


# ============================================================
# 2) Park 法：经典 AX=XB（交叉验证用）
# ============================================================
def make_axxb_pairs(A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """由 A/B 生成 AX=XB 的全部有效位姿对。

    由 X B_i = A_i Y 推出： (A_i A_j^-1) X = X (B_i B_j^-1)。
    """
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    A_pairs, B_pairs = [], []
    for i in range(len(A) - 1):
        for j in range(i + 1, len(A)):
            A_pairs.append(A[i] @ np.linalg.inv(A[j]))
            B_pairs.append(B[i] @ np.linalg.inv(B[j]))
    return np.asarray(A_pairs), np.asarray(B_pairs)


def solve_park_axxb(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Park & Martin 角轴法解 AX=XB，返回 X（4x4）。"""
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    N = len(A)
    # 角轴向量：X beta_j = alpha_j
    P = np.stack([_mat_to_rotvec(B[j][:3, :3]) for j in range(N)], axis=1)  # 3xN
    Q = np.stack([_mat_to_rotvec(A[j][:3, :3]) for j in range(N)], axis=1)  # 3xN
    U, _, Vt = np.linalg.svd(Q @ P.T)
    RX = U @ Vt
    if np.linalg.det(RX) < 0:
        U[:, -1] *= -1.0
        RX = U @ Vt
    # 平移：(RA - I) tX = RX tB - tA
    P2 = np.zeros((3 * N, 3))
    q2 = np.zeros(3 * N)
    for j in range(N):
        P2[3 * j:3 * j + 3] = A[j][:3, :3] - np.eye(3)
        q2[3 * j:3 * j + 3] = RX @ B[j][:3, 3] - A[j][:3, 3]
    tx, *_ = np.linalg.lstsq(P2, q2, rcond=None)
    return homogeneous(RX, tx)


# ============================================================
# 3) 非线性联合精化（scipy least_squares）
# ============================================================
def refine_ax_yb(A: np.ndarray, B: np.ndarray, X0: np.ndarray, Y0: np.ndarray,
                 w_deg: float = 1.0) -> Tuple[np.ndarray, np.ndarray, dict]:
    """对 (X, Y) 联合精化，最小化 X B_i 与 A_i Y 的位置(mm)与旋转(deg)残差。

    参数用 12 维向量：x, y, z, rotvec(X), x, y, z, rotvec(Y)。
    w_deg：1 度旋转残差折算成的 mm 权重（默认 1° ≈ 1mm）。
    """
    from scipy.optimize import least_squares

    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)

    def unpack(p):
        X = homogeneous(_rotvec_to_mat(p[3:6]), p[0:3])
        Y = homogeneous(_rotvec_to_mat(p[9:12]), p[6:9])
        return X, Y

    def resid(p):
        X, Y = unpack(p)
        out = []
        for i in range(len(A)):
            M1 = X @ B[i]
            M2 = A[i] @ Y
            dp = (M1[:3, 3] - M2[:3, 3]) * 1000.0  # mm
            out.extend(dp)
            # SO(3) 对数映射保留旋转误差轴方向；分量单位为度。
            dr = np.degrees(_mat_to_rotvec(M2[:3, :3].T @ M1[:3, :3]))
            out.extend(dr * w_deg)
        return np.asarray(out)

    p0 = np.concatenate([X0[:3, 3], _mat_to_rotvec(X0[:3, :3]),
                         Y0[:3, 3], _mat_to_rotvec(Y0[:3, :3])])
    res = least_squares(resid, p0, method="trf", loss="soft_l1",
                        f_scale=10.0, max_nfev=20000)
    X, Y = unpack(res.x)
    info = {"cost": float(res.cost), "nfev": int(res.nfev),
            "optimality": float(res.optimality), "success": bool(res.success)}
    return X, Y, info


# ============================================================
# 4) 验证
# ============================================================
def verify_ax_yb(A: np.ndarray, B: np.ndarray, X: np.ndarray, Y: np.ndarray) -> dict:
    """返回逐位姿残差与一致性统计。"""
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    pos_mm, rot_deg, Ys = [], [], []
    for i in range(len(A)):
        M1 = X @ B[i]
        M2 = A[i] @ Y
        pos_mm.append(float(np.linalg.norm(M1[:3, 3] - M2[:3, 3]) * 1000.0))
        rot_deg.append(angle_deg(M1[:3, :3], M2[:3, :3]))
        Ys.append(np.linalg.inv(A[i]) @ X @ B[i])
    Ys = np.asarray(Ys)

    # Y 一致性：各姿态反推的 T_flange_board 应几乎相同
    Rm = mean_rotation(Ys[:, :3, :3])
    Ym = homogeneous(Rm, Ys[:, :3, 3].mean(axis=0))
    dy = np.linalg.inv(Ym) @ Ys
    y_pos_std = float(np.linalg.norm(np.std(Ys[:, :3, 3], axis=0)) * 1000.0)
    y_rot_max = float(max(angle_deg(dy[i][:3, :3], np.eye(3)) for i in range(len(dy))))
    y_pos_rmse = float(np.sqrt(np.mean(np.sum((Ys[:, :3, 3] - Ym[:3, 3]) ** 2, axis=1))) * 1000.0)

    # 法兰姿态分散度（手眼标定质量的重要指标）
    axes, angles = [], []
    for i in range(len(A)):
        ax, ang = rotation_axis_angle(A[i][:3, :3])
        axes.append(ax)
        angles.append(ang)
    axes = np.asarray(axes)
    max_span = 0.0
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            c = float(np.clip(np.dot(axes[i], axes[j]), -1.0, 1.0))
            max_span = max(max_span, np.degrees(np.arccos(c)))
    pos_range = np.ptp(A[:, :3, 3], axis=0) * 1000.0  # mm

    return {
        "pos_residual_mm": np.asarray(pos_mm),
        "rot_residual_deg": np.asarray(rot_deg),
        "pos_residual_mean_mm": float(np.mean(pos_mm)),
        "pos_residual_max_mm": float(np.max(pos_mm)),
        "rot_residual_mean_deg": float(np.mean(rot_deg)),
        "rot_residual_max_deg": float(np.max(rot_deg)),
        "y_pose_std_mm": y_pos_std,
        "y_pose_rmse_mm": y_pos_rmse,
        "y_rotation_spread_deg": y_rot_max,
        "flange_rotation_span_deg": max_span,
        "flange_pos_range_mm": pos_range,
        "n": len(A),
    }


# ============================================================
# 5) 自检
# ============================================================
def leave_one_out_ax_yb(A: np.ndarray, B: np.ndarray, do_refine: bool = True) -> dict:
    """留一法精度验算（手眼标定精度估计）。

    每次去掉一个位姿，用其余位姿重新标定 (X, Y)，
    再用 X·B = A·Y 预测被移除位姿的板位姿 B_pred = X^-1·A·Y，
    与真实 B 比较位置/旋转误差。返回逐点误差与统计。

    这是对“标定结果对新位姿的预测能力”的估计，比全量残差更能反映真实精度。
    """
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    n = len(A)
    if n < 4:
        raise ValueError(f"留一法至少需要 4 个位姿，当前 {n}")
    pos_errors, rot_errors = [], []
    for i in range(n):
        keep = [j for j in range(n) if j != i]
        X0, Y0 = solve_shah_robot_world(A[keep], B[keep])
        if do_refine:
            X, Y, _ = refine_ax_yb(A[keep], B[keep], X0, Y0)
        else:
            X, Y = X0, Y0
        B_pred = np.linalg.inv(X) @ A[i] @ Y  # 预测板位姿
        pos_errors.append(float(np.linalg.norm(B_pred[:3, 3] - B[i][:3, 3]) * 1000.0))
        rot_errors.append(angle_deg(B_pred[:3, :3], B[i][:3, :3]))
    pos_errors = np.asarray(pos_errors)
    rot_errors = np.asarray(rot_errors)
    return {
        "pos_error_mm": pos_errors,
        "rot_error_deg": rot_errors,
        "pos_mean_mm": float(pos_errors.mean()),
        "pos_max_mm": float(pos_errors.max()),
        "rot_mean_deg": float(rot_errors.mean()),
        "rot_max_deg": float(rot_errors.max()),
        "n": n,
    }


# ============================================================
# 5) 眼在手上（Eye-in-Hand）求解 / 验证
#    相机装在法兰上随动，标定板固定在工作台。
#    X = T_tcp_camera（camera -> 当前机器人 TCP，待求）
#    约束：T_base_board = G_i @ X @ H_i 恒定，G=T_base_tcp, H=T_camera_board
# ============================================================
def make_axxb_pairs_eye_in_hand(G: np.ndarray, H: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """由 G/H 生成 AX=XB 的全部有效位姿对。

    T_base_board = G_i X H_i = G_j X H_j
      => (G_j^-1 G_i) X = X (H_j H_i^-1)   =>  A X = X B
    """
    G = np.asarray(G, dtype=np.float64)
    H = np.asarray(H, dtype=np.float64)
    A_pairs, B_pairs = [], []
    for i in range(len(G) - 1):
        for j in range(i + 1, len(G)):
            A_pairs.append(np.linalg.inv(G[j]) @ G[i])
            B_pairs.append(H[j] @ np.linalg.inv(H[i]))
    return np.asarray(A_pairs), np.asarray(B_pairs)


def refine_axxb(A: np.ndarray, B: np.ndarray, X0: np.ndarray,
                w_deg: float = 1.0) -> Tuple[np.ndarray, dict]:
    """对经典 AX=XB 做非线性精化：最小化 A_i X 与 X B_i 的差异。

    A, B: 位姿对（由 make_axxb_pairs_eye_in_hand 生成）；X = T_tcp_camera。
    """
    from scipy.optimize import least_squares

    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)

    def unpack(p):
        return homogeneous(_rotvec_to_mat(p[3:6]), p[0:3])

    def resid(p):
        X = unpack(p)
        out = []
        for i in range(len(A)):
            M1 = A[i] @ X
            M2 = X @ B[i]
            out.extend((M1[:3, 3] - M2[:3, 3]) * 1000.0)  # mm
            dr = np.degrees(_mat_to_rotvec(M2[:3, :3].T @ M1[:3, :3]))
            out.extend(dr * w_deg)
        return np.asarray(out)

    p0 = np.concatenate([X0[:3, 3], _mat_to_rotvec(X0[:3, :3])])
    res = least_squares(resid, p0, method="trf", loss="soft_l1",
                        f_scale=10.0, max_nfev=20000)
    info = {"cost": float(res.cost), "nfev": int(res.nfev), "success": bool(res.success)}
    return unpack(res.x), info


def refine_eye_in_hand(G: np.ndarray, H: np.ndarray, X0: np.ndarray,
                       w_deg: float = 10.0) -> Tuple[np.ndarray, dict]:
    """眼在手上非线性精化：直接最小化“标定板在基座系一致性”指标。

    目标 = Σ_i || G_i·X·H_i − mean(T_base_board) ||，位置(mm) +
    SO(3) 三维对数旋转残差(度×w_deg)。
    与 verify_eye_in_hand 显示的指标一致，因此精化必然降低（或不劣于）该指标。
    w_deg：旋转权重。1° 旋转在 ~700mm 工作距离≈12mm 物理误差，故默认 10 较合理
    （位置与旋转在报告里分别显示，精化会同时兼顾）。
    """
    from scipy.optimize import least_squares

    G = np.asarray(G, dtype=np.float64)
    H = np.asarray(H, dtype=np.float64)
    n = len(G)

    def unpack(p):
        return homogeneous(_rotvec_to_mat(p[3:6]), p[0:3])

    def resid(p):
        X = unpack(p)
        Tbb = np.asarray([G[i] @ X @ H[i] for i in range(n)])
        Rm = mean_rotation(Tbb[:, :3, :3])
        Tm = homogeneous(Rm, Tbb[:, :3, 3].mean(axis=0))
        rel = np.linalg.inv(Tm) @ Tbb
        out = []
        for i in range(n):
            out.extend(rel[i][:3, 3] * 1000.0)                       # mm
            dr = np.degrees(_mat_to_rotvec(rel[i][:3, :3]))
            out.extend(dr * w_deg)                                  # 三维旋转残差(度)
        return np.asarray(out)

    p0 = np.concatenate([X0[:3, 3], _mat_to_rotvec(X0[:3, :3])])
    res = least_squares(resid, p0, method="trf", loss="soft_l1",
                        f_scale=10.0, max_nfev=20000)
    info = {"cost": float(res.cost), "nfev": int(res.nfev), "success": bool(res.success)}
    return unpack(res.x), info


def solve_eye_in_hand(G: np.ndarray, H: np.ndarray, do_refine: bool = True
                      ) -> Tuple[np.ndarray, np.ndarray, dict]:
    """眼在手上标定：X = T_tcp_camera。

    G: (N,4,4) T_base_tcp；H: (N,4,4) T_camera_board（固定板，PnP 得）。
    Park 线性解 + 可选非线性精化（目标=标定板一致性，与验证指标一致）。
    返回 (X, X_linear, info)。
    """
    A, B = make_axxb_pairs_eye_in_hand(G, H)
    X0 = solve_park_axxb(A, B)
    if do_refine:
        X, info = refine_eye_in_hand(G, H, X0)
    else:
        X, info = X0, {}
    return X, X0, info


def verify_eye_in_hand(G: np.ndarray, H: np.ndarray, X: np.ndarray) -> dict:
    """眼在手上验证：T_base_board_i = G_i X H_i 应几乎恒定。"""
    G = np.asarray(G, dtype=np.float64)
    H = np.asarray(H, dtype=np.float64)
    Tbb = np.asarray([G[i] @ X @ H[i] for i in range(len(G))])
    # 平均 T_base_board
    Rm = mean_rotation(Tbb[:, :3, :3])
    Tm = homogeneous(Rm, Tbb[:, :3, 3].mean(axis=0))
    rel = np.linalg.inv(Tm) @ Tbb
    pos_mm = np.linalg.norm(rel[:, :3, 3], axis=1) * 1000.0
    rot_deg = np.asarray([angle_deg(rel[i][:3, :3], np.eye(3)) for i in range(len(rel))])
    # 法兰姿态分散度
    axes = np.asarray([rotation_axis_angle(G[i][:3, :3])[0] for i in range(len(G))])
    max_span = 0.0
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            c = float(np.clip(np.dot(axes[i], axes[j]), -1.0, 1.0))
            max_span = max(max_span, np.degrees(np.arccos(c)))
    pos_range = np.ptp(G[:, :3, 3], axis=0) * 1000.0
    return {
        "pos_residual_mm": pos_mm,
        "rot_residual_deg": rot_deg,
        "pos_residual_mean_mm": float(pos_mm.mean()),
        "pos_residual_max_mm": float(pos_mm.max()),
        "rot_residual_mean_deg": float(rot_deg.mean()),
        "rot_residual_max_deg": float(rot_deg.max()),
        "board_pose_std_mm": float(np.linalg.norm(np.std(Tbb[:, :3, 3], axis=0)) * 1000.0),
        "board_rotation_spread_deg": float(max(rot_deg)),
        "flange_rotation_span_deg": max_span,
        "flange_pos_range_mm": pos_range,
        "n": len(G),
    }


def leave_one_out_eye_in_hand(G: np.ndarray, H: np.ndarray, do_refine: bool = True) -> dict:
    """眼在手上留一法精度验算：去掉一个位姿重标定，预测其板位姿并比较。"""
    G = np.asarray(G, dtype=np.float64)
    H = np.asarray(H, dtype=np.float64)
    n = len(G)
    if n < 4:
        raise ValueError(f"留一法至少需要 4 个位姿，当前 {n}")
    pos_errors, rot_errors = [], []
    for i in range(n):
        keep = [j for j in range(n) if j != i]
        X, _, _ = solve_eye_in_hand(G[keep], H[keep], do_refine=do_refine)
        Tbb_keep = np.asarray([G[j] @ X @ H[j] for j in keep])
        Rm = mean_rotation(Tbb_keep[:, :3, :3])
        Tm = homogeneous(Rm, Tbb_keep[:, :3, 3].mean(axis=0))
        H_pred = np.linalg.inv(G[i] @ X) @ Tm  # 预测 T_cam_board
        pos_errors.append(float(np.linalg.norm(H_pred[:3, 3] - H[i][:3, 3]) * 1000.0))
        rot_errors.append(angle_deg(H_pred[:3, :3], H[i][:3, :3]))
    pos_errors = np.asarray(pos_errors)
    rot_errors = np.asarray(rot_errors)
    return {
        "pos_error_mm": pos_errors,
        "rot_error_deg": rot_errors,
        "pos_mean_mm": float(pos_errors.mean()),
        "pos_max_mm": float(pos_errors.max()),
        "rot_mean_deg": float(rot_errors.mean()),
        "rot_max_deg": float(rot_errors.max()),
        "n": n,
    }


def selftest(seed: int = 0) -> dict:
    """合成数据验证：随机 X, Y, A_i，构造 B_i = X^-1 A_i Y，应能精确恢复 X。"""
    rng = np.random.default_rng(seed)
    N = 14

    def rand_pose():
        ax = rng.normal(size=3)
        ax = ax / np.linalg.norm(ax) * rng.uniform(20, 60)
        R = _rotvec_to_mat(np.radians(ax))
        t = rng.uniform(-0.3, 0.3, 3)
        return homogeneous(R, t)

    X_gt = rand_pose()
    Y_gt = rand_pose()
    A = np.asarray([rand_pose() for _ in range(N)])
    B = np.asarray([np.linalg.inv(X_gt) @ A[i] @ Y_gt for i in range(N)])

    X0, Y0 = solve_shah_robot_world(A, B)
    Ap, Bp = make_axxb_pairs(A, B)
    X_park = solve_park_axxb(Ap, Bp)
    X, Y, info = refine_ax_yb(A, B, X0, Y0)

    def pos_err(M, ref):
        return float(np.linalg.norm(M[:3, 3] - ref[:3, 3]) * 1000.0)

    def rot_err(M, ref):
        return angle_deg(M[:3, :3], ref[:3, :3])

    result = {
        "X_shah_pos_mm": pos_err(X0, X_gt),
        "X_shah_rot_deg": rot_err(X0, X_gt),
        "X_park_pos_mm": pos_err(X_park, X_gt),
        "X_park_rot_deg": rot_err(X_park, X_gt),
        "X_refined_pos_mm": pos_err(X, X_gt),
        "X_refined_rot_deg": rot_err(X, X_gt),
        "Y_refined_pos_mm": pos_err(Y, Y_gt),
        "Y_refined_rot_deg": rot_err(Y, Y_gt),
        "verify": verify_ax_yb(A, B, X, Y),
    }
    return result


if __name__ == "__main__":
    r = selftest()
    for k, v in r.items():
        if isinstance(v, dict):
            print(f"{k}:")
            for kk, vv in v.items():
                if isinstance(vv, np.ndarray):
                    print(f"   {kk}: {np.round(vv, 4)}")
                else:
                    print(f"   {kk}: {vv}")
        else:
            print(f"{k}: {v}")
