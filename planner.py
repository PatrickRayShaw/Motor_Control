"""
轮廓跟随 — 运动规划器 (v2, 适配严格 amax 速度控制器)
输入轮廓点序列 → 输出 XY 目标速度序列, 保证每帧单轴变化 ≤ amax·dt (物理可行)

【完整流程】
  Step 1  轮廓点序列 (x,y)[]          — 随机生成 / --input 读 CSV / 自己给
  Step 2  弧长参数化 (Segment/Path)   — 路径 → 弧长 s 上的位置与切线
  Step 3  速度剖面 (build_velocity_profile)
           曲率限速 v≤√(amax/κ) + 加速约束 + 逆扫减速; 闭合轮廓自动识别, 终点不减速
  Step 4  运动生成 (plan_motion)      — 1ms/帧的 vx/vy 目标序列
  Step 5  闭环仿真 (motor_controller) — S曲线+前馈+PID 双轴跟随, 出图与误差统计

【运行】
  python planner.py                仿真一条随机轮廓 → pipeline_Random.png
  python planner.py --input a.csv  从 CSV 读点序列 (2 列 x,y; 或 7 列 export 格式回读)
  python planner.py --seed N       固定随机种子, 结果可复现
  python planner.py --step 2|3|4   只看指定步骤, 提前退出
  python planner.py --simple       简化一阶电机模型对照 (不用 S 曲线控制器)
  python planner.py --export       额外导出 target_velocities_*.csv 给嵌入式侧

【改电机参数】main() 里:
  V_MAX = 500   直道最高速度 (mm/s)
  AMAX  = 1000  最大加速度 (mm/s²) ← 必须与 motor_controller.py 的 amax 一致
  DT    = 0.001 控制周期 (s)       ← 必须与电机控制循环周期一致
  TAU   = 0.005 电机时间常数 (s)
  KP    = 5.0   控制器 P 增益

【嵌入式使用】
  控制器每 DT 秒调用一次, 双轴各一个实例:
    v_ref_x, cmd_x = mc_x.update(vx_target[i], encoder_x())
    v_ref_y, cmd_y = mc_y.update(vy_target[i], encoder_y())
  vx/vy 目标序列来自 --export 的 CSV (第 4/5 列), 或在嵌入式侧直接调本模块的
  build_path → build_velocity_profile → plan_motion 实时生成。
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import motor_controller



def random_contour_points(n_vert=None, seed=None):
    """
    随机圆角闭合轮廓: n_vert 个随机顶点 (绕圆心排列, 不自交) + 自适应圆角

    圆角半径自适应: ≤10mm, 且不超过相邻边的一半 (防止圆弧吃掉整条边)。
    返回密集点序列 (直线段 ~1mm 间隔, 圆弧按角度采样)。
    """
    rng = np.random.default_rng(seed)
    if n_vert is None:
        n_vert = int(rng.integers(4, 8))
    ang = np.sort(rng.uniform(0, 2 * np.pi, n_vert))
    rad = rng.uniform(30, 70, n_vert)
    verts = np.column_stack([50 + rad * np.cos(ang), 50 + rad * np.sin(ang)])

    n = n_vert
    beta = np.zeros(n)
    tin, tout = np.zeros((n, 2)), np.zeros((n, 2))
    r_c = np.zeros(n)
    for i in range(n):
        P, V, Q = verts[i - 1], verts[i], verts[(i + 1) % n]
        u1 = (P - V) / np.linalg.norm(P - V)
        u2 = (Q - V) / np.linalg.norm(Q - V)
        beta[i] = max(np.arccos(np.clip(np.dot(u1, u2), -1, 1)) / 2.0, 1e-3)
        r_c[i] = min(10.0, 0.49 * min(np.linalg.norm(P - V),
                                      np.linalg.norm(V - Q)) * np.tan(beta[i]))
        d = r_c[i] / np.tan(beta[i])
        tin[i] = V + d * u1
        tout[i] = V + d * u2

    pts = []
    for i in range(n):
        V, P, Q = verts[i], verts[i - 1], verts[(i + 1) % n]
        a, b = tout[i - 1], tin[i]          # 边 (P→V): 上顶点出切点 → 本顶点入切点
        L = np.linalg.norm(b - a)
        m = max(1, int(L))
        for k in range(1, m + 1):
            pts.append((a[0] + (b[0] - a[0]) * k / m, a[1] + (b[1] - a[1]) * k / m))
        u1 = (P - V) / np.linalg.norm(P - V)
        u2 = (Q - V) / np.linalg.norm(Q - V)
        bis = u1 + u2
        bn = np.linalg.norm(bis)
        if bn < 1e-9:
            bis = np.array([-u1[1], u1[0]])
        else:
            bis /= bn
        center = V + (r_c[i] / np.sin(beta[i])) * bis
        a0 = np.arctan2(tin[i][1] - center[1], tin[i][0] - center[0])
        a1 = np.arctan2(tout[i][1] - center[1], tout[i][0] - center[0])
        sweep = (a1 - a0 + np.pi) % (2 * np.pi) - np.pi
        n_arc = max(4, int(abs(sweep) / (np.pi / 40)))
        for k in range(1, n_arc + 1):
            a_k = a0 + sweep * k / n_arc
            pts.append((center[0] + r_c[i] * np.cos(a_k), center[1] + r_c[i] * np.sin(a_k)))
    return pts




class Segment:
    """一条直线段: eval(u) → (x, y, tx, ty)"""
    __slots__ = ('x0', 'y0', 'x1', 'y1', 'L', 'tx', 'ty')

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0 = x0, y0
        self.x1, self.y1 = x1, y1
        dx, dy = x1 - x0, y1 - y0
        self.L = np.hypot(dx, dy)
        if self.L > 1e-12:
            self.tx, self.ty = dx / self.L, dy / self.L
        else:
            self.tx, self.ty = 1.0, 0.0

    def eval(self, u):
        if self.L < 1e-12:
            return self.x0, self.y0, self.tx, self.ty
        t = u / self.L
        return (self.x0 + t * (self.x1 - self.x0),
                self.y0 + t * (self.y1 - self.y0),
                self.tx, self.ty)


class Path:
    """路径拼接器: 累积弧长表, eval(s) → (x, y, tx, ty)"""
    __slots__ = ('segs', 'cum', 'total')

    def __init__(self, segs):
        self.segs = segs
        self.cum = [0.0]
        for seg in segs:
            self.cum.append(self.cum[-1] + seg.L)
        self.total = self.cum[-1]

    def eval(self, s):
        s = np.clip(s, 0, self.total)
        for i, seg in enumerate(self.segs):
            if s <= self.cum[i + 1]:
                return seg.eval(s - self.cum[i])
        return self.segs[-1].eval(self.segs[-1].L)


def read_points_csv(path):
    """
    从 CSV 读轮廓点序列:
      2 列 (x,y)            — 自己/同事导出的点
      7 列 (export 格式)    — planner --export 的输出, 取最后两列 x,y 回读
    表头行 / # 注释行自动跳过。UTF-8 (含 BOM) 均可。
    """
    data = np.genfromtxt(path, delimiter=',', comments='#', encoding='utf-8-sig')
    data = data[~np.isnan(data).any(axis=1)]
    if data.shape[0] == 0:
        raise ValueError(f'{path}: 没有读到数据')
    if data.shape[1] == 2:
        pts = data
    elif data.shape[1] >= 6:
        pts = data[:, -2:]
    else:
        raise ValueError(f'{path}: 无法识别点序列格式 ({data.shape[1]} 列), 需要 2 列 (x,y)')
    return [(float(x), float(y)) for x, y in pts]


def build_path(points):
    """点序列 → Path 对象"""
    segs = [Segment(points[i][0], points[i][1],
                    points[i + 1][0], points[i + 1][1])
            for i in range(len(points) - 1)]
    return Path(segs)



# 运动参数 — 改你电机的实际值
SMOOTH_MM = 2.0   # 方向平滑窗口 (mm): 越小越贴路径, 越大越平滑
                  # 2026-08-12 5→2: 闭合轮廓 dist-path max 0.4235→0.2226 (seed=1 实测)
DS_GRID = 0.5     # 弧长采样步长 (mm)
KAPPA_CLIP = 2.0  # 曲率上限 (1/mm): 超过则截断 (防病态数值)。0.5 会把
                  # r<2mm 光滑发夹弯的真实 κ 截掉 → 限速虚高, 2026-08-13 提到 2.0。
CUSP_THRESH_RAD = 1.3  # 相邻网格采样切线角差 (wrapped) 超过此值 → 方向不连续
                       # 回折点 (如 Lissajous 8 字尖点): 必须减速到 ~0 原地反向。
                       # 光滑段每采样 Δθ≈κ·ds≤0.1, 尖点处 ≈π, 阈值安全。
CUSP_V_MIN = 4.0       # 回折点最低速度 (mm/s): 方向阶跃必须在近零速完成,
                       # 且反向帧增量 2·v·|cosθ| ≤ amax·dt (4·2·0.11=0.88<1.0)
                       # 保证离散推进能越过回折点且单帧不超限。


def _arc_grid(path, ds=DS_GRID):
    """弧长均匀网格 + 每点 (x, y, 切线角)"""
    n = max(20, int(np.ceil(path.total / ds)))
    s_grid = np.linspace(0, path.total, n)
    x = np.zeros(n); y = np.zeros(n); th = np.zeros(n)
    for i, si in enumerate(s_grid):
        xi, yi, txi, tyi = path.eval(si)
        x[i], y[i] = xi, yi
        th[i] = np.arctan2(tyi, txi)
    return s_grid, x, y, th


def _smooth_theta(theta, win, is_closed=False):
    """弧长域滑动平均"""
    if win <= 1:
        return theta
    n = len(theta)
    pad = win // 2
    if is_closed:
        trend = np.linspace(0, theta[-1] - theta[0], n)
        th_detrend = theta - trend
        th_pad = np.pad(th_detrend, pad, mode='wrap')
        th_s = np.array([np.mean(th_pad[i:i + win])
                         for i in range(n)]) + trend
    else:
        th_pad = np.pad(theta, pad, mode='edge')
        th_s = np.array([np.mean(th_pad[i:i + win]) for i in range(n)])
    return th_s


def _smooth_theta_split(theta, win, cusp_cells, is_closed):
    """
    按回折点分段的方向平滑: 每段独立边缘平滑, 回折点两侧方向不混合.

    回折点 (方向不连续) 处若让平滑窗口跨过尖点, 会得到两侧方向的
    "平均角" (如 −83.7° 与 +96.3° 平均成 ~6° 的横切方向), 目标速度会
    横着冲出路径。分段后每段保持自身方向, 反向在回折点处阶跃
    (配合 v→CUSP_V_MIN 原地反向, 见 _advance 的阶跃插值)。
    """
    if cusp_cells is None or len(cusp_cells) == 0:
        return _smooth_theta(theta, win, is_closed)
    n = len(theta)
    boundaries = np.concatenate([[0], np.sort(cusp_cells) + 1, [n]])
    out = np.empty_like(theta)
    for k in range(len(boundaries) - 1):
        a0, a1 = boundaries[k], boundaries[k + 1]
        if a1 - a0 <= 1:
            out[a0:a1] = theta[a0:a1]
        elif k == 0 and is_closed:
            # 环绕段 (跨 s=0 的回折点之间): 0 处方向连续, 作为闭环整体平滑
            seg = np.concatenate([theta[a0:], theta[:a1]])
            sm = _smooth_theta(seg, win, True)
            out[a0:] = sm[:n - a0]
            out[:a1] = sm[n - a0:]
        else:
            out[a0:a1] = _smooth_theta(theta[a0:a1], win, False)
    return out


def build_velocity_profile(path, v_max, amax, n_samples=None):
    """
    Step 3: 弧长域速度剖面 (每帧单轴 |Δvx|,|Δvy| ≤ amax·dt 由构造保证).

    返回:
      s_samples[]   : 弧长采样位置 (mm)
      v_limits[]    : 最终限速 (mm/s)
      v_curvature_only[] : 曲率+加速约束 (不含逆扫) — 画图对照
    """
    L = path.total
    s_grid, xg, yg, th_raw = _arc_grid(path)
    n = len(s_grid)

    seg_len = L / max(1, len(path.segs))
    is_closed = np.hypot(xg[0] - xg[-1], yg[0] - yg[-1]) < max(2 * seg_len,
                                                                L * 0.01)

    th_unw = np.unwrap(th_raw)

    # 回折点检测: 相邻网格采样点切线角 (wrapped) 跳变超过阈值 → 方向不连续
    dth_raw = np.abs((np.diff(th_raw) + np.pi) % (2 * np.pi) - np.pi)
    cusp_cells = np.where(dth_raw > CUSP_THRESH_RAD)[0]

    win = max(3, int(round(SMOOTH_MM / (L / (n - 1)))))
    if win % 2 == 0:
        win += 1
    theta_s = _smooth_theta_split(th_unw, win, cusp_cells, is_closed)
    theta_out = theta_s

    dth = np.abs(np.gradient(theta_s, s_grid))
    kappa = np.minimum(dth, KAPPA_CLIP)
    if len(cusp_cells):
        # 回折点处 θ_s 阶跃产生的 κ 尖峰不代表旋转成本: 方向反向由减速到
        # ~0 原地完成, 不占 v²κ 预算 (否则 a_avail=0 → v 冻结 → 冲过回折点)。
        # np.gradient 在阶跃处泄漏 2 个样本。
        for ci in cusp_cells:
            kappa[ci:ci + 2] = 0.0

    ds = s_grid[1] - s_grid[0]

    v_curve = np.full(n, v_max, dtype=float)
    m = kappa > 1e-9
    v_curve[m] = np.minimum(v_curve[m], np.sqrt(amax / kappa[m]))

    v_acc = np.zeros(n)
    v_prev = 0.0
    for i in range(n):
        a_avail = amax - kappa[i] * v_prev * v_prev
        if a_avail < 1e-9:
            v_acc[i] = v_curve[i]
        else:
            v_acc[i] = min(np.sqrt(v_prev * v_prev + 2 * a_avail * ds),
                           v_curve[i])
        v_prev = v_acc[i]
    v_lim = np.minimum(v_curve, v_acc)
    if len(cusp_cells):
        # 回折点最低速度地板: 方向阶跃必须在此完成 (见 _advance 阶跃插值),
        # 逆扫从地板向两侧推出 amax 锥形减速 (√(2·amax·d))。
        for ci in cusp_cells:
            v_lim[ci] = CUSP_V_MIN
            v_lim[min(ci + 1, n - 1)] = CUSP_V_MIN

    if is_closed:
        for _ in range(n):
            changed = False
            for i in range(n):
                j = (i + 1) % n
                k_i = kappa[i]
                v_dec = np.sqrt((v_lim[j] ** 2 + 2 * amax * ds)
                                / (1 + 2 * k_i * ds))
                if v_dec < v_lim[i]:
                    v_lim[i] = v_dec
                    changed = True
            if not changed:
                break
    else:
        v_lim[-1] = 0.0
        for i in range(n - 2, -1, -1):
            k_i = kappa[i]
            v_dec = np.sqrt((v_lim[i + 1] ** 2 + 2 * amax * ds)
                            / (1 + 2 * k_i * ds))
            if v_dec < v_lim[i]:
                v_lim[i] = v_dec

    v_curvature_only = np.minimum(v_curve, v_acc).copy()

    profile = {
        's': s_grid, 'theta': theta_out, 'kappa': kappa,
        'is_closed': is_closed, 'L': L,
        'cusp_cells': cusp_cells,
    }
    build_velocity_profile._last = profile
    return s_grid, v_lim, v_curvature_only




def plan_motion(path, s_samples, v_limits, v_max, amax, dt=0.001,
                smooth_ms=15, max_iter=3):
    """
    Step 4: 生成 XY 目标速度序列 (单轴每帧 ≤ amax·dt, 物理可行).

    返回:
      t_seq[], v_seq[], vx_seq[], vy_seq[], x_seq[], y_seq[]
    """
    prof = build_velocity_profile._last
    if prof is None or prof['s'] is not s_samples:
        build_velocity_profile(path, v_max, amax)
        prof = build_velocity_profile._last
    theta_s = prof['theta']
    is_closed = prof['is_closed']
    L = path.total
    cusp_cells = prof.get('cusp_cells')

    v_lim = v_limits.copy()
    kappa = prof['kappa']
    n_ok = False
    for _ in range(max_iter):
        t_seq, v_seq, vx_seq, vy_seq, x_seq, y_seq = _advance(
            path, s_samples, v_lim, theta_s, kappa, is_closed, v_max,
            amax, dt, cusp_cells)
        a_lim = amax * dt
        dvx = np.abs(np.diff(vx_seq, prepend=vx_seq[0]))
        dvy = np.abs(np.diff(vy_seq, prepend=vy_seq[0]))
        over = np.maximum(dvx, dvy) / a_lim
        bad = np.where(over > 1.0001)[0]
        if len(bad) == 0:
            n_ok = True
            break
        s_bad = np.interp(bad, np.arange(len(v_seq)), np.cumsum(
            np.hypot(np.diff(x_seq, prepend=x_seq[0]),
                     np.diff(y_seq, prepend=y_seq[0]))))
        for si, oi in zip(s_bad, over[bad]):
            idx = np.abs(s_samples - si).argmin()
            w = max(1, int(round(SMOOTH_MM / (s_samples[1] - s_samples[0]))))
            lo, hi = max(0, idx - w), min(len(v_lim), idx + w)
            v_lim[lo:hi] = np.minimum(v_lim[lo:hi],
                                      v_lim[lo:hi] / np.sqrt(oi))

    if not n_ok:
        print(f'  [warn] 物理校验未完全收敛 (剩余 {len(bad)} 帧超标)')

    return t_seq, v_seq, vx_seq, vy_seq, x_seq, y_seq


def _advance(path, s_samples, v_lim, theta_s, kappa, is_closed, v_max, amax,
             dt, cusp_cells=None):
    """
    弧长推进 + 切线分解生成 vx/vy 序列。

    s_hat = 实际位置弧长 (mm): 方向场/曲率/速度剖面都索引在实际位置,
    而不是"理想推进"的弧长。钳制帧 (单轴 |Δv| 超限被缩放) 使命令速度
    幅值 < v_seq, 实际沿路径推进 < v·dt → 若方向仍挂在前方理想弧长上,
    运动会以 κ·Δs 的切角切入弯道内侧 (Lissajous 顶点 2.26mm 横向误差
    根因, 2026-08-13 修复)。每帧钳制后按 命令速度·路径切线 重新对账。
    """
    L = path.total
    steps = int(L / (v_max * 0.3) * 2 / dt) * 2

    s_hat = 0.0
    v = 0.0
    t_seq, v_seq = [], []
    vx_seq, vy_seq = [], []
    x_seq, y_seq = [], []
    a_lim = amax * dt
    if cusp_cells is not None and len(cusp_cells) > 0:
        cusp_set = set(int(ci) for ci in cusp_cells)
    else:
        cusp_set = None

    for _ in range(steps):
        s_next = s_hat + v * dt
        v_target = np.interp(s_next, s_samples, v_lim)
        if not is_closed:
            v_target = min(v_target, np.sqrt(max(0.0, 2 * amax * (L - s_next))))
        dv = v_target - v
        k_s = np.interp(s_hat, s_samples, kappa)
        a_tan = max(0.0, amax - k_s * v * v) * dt
        a_dec = max(0.2 * amax, amax - k_s * v * v) * dt
        v += min(max(dv, -a_dec), a_tan)

        s_hat += v * dt
        if s_hat >= L:
            s_hat = L
            if is_closed:
                pass
            else:
                v = 0.0

        if cusp_set is not None:
            i_cell = int(np.searchsorted(s_samples, s_hat, side='right')) - 1
            i_cell = min(max(i_cell, 0), len(s_samples) - 2)
            if i_cell in cusp_set:
                # 回折单元内保持入侧方向, 出单元瞬间阶跃反向 (此时 v≈CUSP_V_MIN,
                # 单帧 Δv=2·v·|cosθ|≤amax·dt, 方向始终贴路径, 无横切漂移)
                th = theta_s[i_cell]
            else:
                th = np.interp(s_hat, s_samples, theta_s)
        else:
            th = np.interp(s_hat, s_samples, theta_s)
        vx = v * np.cos(th)
        vy = v * np.sin(th)
        if vx_seq:
            # 单轴硬约束: 分解后 Δvx/Δvy 超限则按比例缩本帧增量 (保持方向)
            dvx = vx - vx_seq[-1]
            dvy = vy - vy_seq[-1]
            dv_max = max(abs(dvx), abs(dvy))
            if dv_max > a_lim:
                f = a_lim / dv_max
                vx = vx_seq[-1] + dvx * f
                vy = vy_seq[-1] + dvy * f
            # 弧长对账: 命令速度在路径切线 (≈(cosθ,sinθ)) 上的投影才是实际
            # 沿路径推进。未钳制帧投影 = v·(cos²+sin²) = v, 对账修正恒为 0;
            # 钳制帧 (f<1) 实际推进 < v·dt, 方向场跟着落回实际位置, 不再
            # 超前指向 → 消除切弯道内侧的横向漂移。
            s_hat += (vx * np.cos(th) + vy * np.sin(th) - v) * dt

        t_seq.append(len(t_seq) * dt)
        v_seq.append(v)
        vx_seq.append(vx)
        vy_seq.append(vy)
        x, y, _, _ = path.eval(s_hat)
        x_seq.append(x)
        y_seq.append(y)

        # 终点容差: 尾段 v_target≈63·(L−s_hat) 使 s_hat 几何逼近 L, 最后
        # v·dt 小于 double ulp → s_hat 冻结在 L−1e-12 永远触发不了 s_hat>=L,
        # 开放路径会 pad 出十几秒零速帧。1e-6mm 远小于任何实际距离, 且
        # v<0.01 保证没有截断风险。v==0 且已离起点: 剖面已终止 (如逆扫把
        # 尾段压到 0), 同样应停。
        if (s_hat >= L - 1e-6 and not is_closed and v < 0.01) or \
           (s_hat >= L and is_closed) or \
           (not is_closed and v == 0.0 and s_hat > 1e-6):
            break

    return (np.array(t_seq), np.array(v_seq),
            np.array(vx_seq), np.array(vy_seq),
            np.array(x_seq), np.array(y_seq))




def simulate_simple_motor(vx_seq, vy_seq, x_seq, y_seq, dt, tau):
    """简化一阶电机模型 (最坏情况对照)"""
    vx_act, vy_act = 0.0, 0.0
    log_xa, log_ya = [x_seq[0]], [y_seq[0]]
    px_act, py_act = x_seq[0], y_seq[0]
    for i in range(len(vx_seq)):
        vx_act += (vx_seq[i] - vx_act) / tau * dt
        vy_act += (vy_seq[i] - vy_act) / tau * dt
        px_act += vx_act * dt
        py_act += vy_act * dt
        log_xa.append(px_act)
        log_ya.append(py_act)
    return np.array(log_xa[:-1]), np.array(log_ya[:-1])


def simulate_with_motor_controller(vx_seq, vy_seq, x_seq, y_seq, dt,
                                   amax, tau, kp=5.0, ki=0.02, kd=0.01,
                                   kff_acc=None):
    """Step 5: 双轴 S 曲线控制器闭环仿真"""
    if kff_acc is None:
        kff_acc = tau
    mc_x = motor_controller.MotorController(
        kp=kp, ki=ki, kd=kd, dt=dt, amax=amax,
        kff_vel=1.0, kff_acc=kff_acc)
    mc_y = motor_controller.MotorController(
        kp=kp, ki=ki, kd=kd, dt=dt, amax=amax,
        kff_vel=1.0, kff_acc=kff_acc)

    n = len(vx_seq)
    vx_ref_log = np.zeros(n)
    vy_ref_log = np.zeros(n)
    vx_act_log = np.zeros(n)
    vy_act_log = np.zeros(n)

    vx_a, vy_a = 0.0, 0.0
    px, py = x_seq[0], y_seq[0]
    log_xa, log_ya = [px], [py]

    for i in range(n):
        vx_ref_log[i], cmd_x = mc_x.update(vx_seq[i], vx_a)
        vy_ref_log[i], cmd_y = mc_y.update(vy_seq[i], vy_a)
        vx_a += (cmd_x - vx_a) / tau * dt
        vy_a += (cmd_y - vy_a) / tau * dt
        vx_act_log[i], vy_act_log[i] = vx_a, vy_a
        px += vx_a * dt
        py += vy_a * dt
        log_xa.append(px)
        log_ya.append(py)

    return (np.array(log_xa[:-1]), np.array(log_ya[:-1]),
            vx_ref_log, vy_ref_log, vx_act_log, vy_act_log)


def point_to_path_distance(xa, ya, path):
    """真轮廓误差: 每个实际位置点到路径的最短距离 (mm)"""
    segs = path.segs
    n = len(xa)
    d_min = np.full(n, np.inf)
    for seg in segs:
        dx = seg.x1 - seg.x0
        dy = seg.y1 - seg.y0
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            continue
        t = ((xa - seg.x0) * dx + (ya - seg.y0) * dy) / L2
        t = np.clip(t, 0.0, 1.0)
        px = seg.x0 + t * dx
        py = seg.y0 + t * dy
        np.minimum(d_min, np.hypot(xa - px, ya - py), out=d_min)
    return d_min




def plot_results(path, s_samples, v_limits, v_curvature_only,
                 t_seq, v_seq, vx_seq, vy_seq, x_seq, y_seq,
                 xa, ya, amax, name,
                 vx_ref=None, vy_ref=None, vx_act=None, vy_act=None):
    prof = build_velocity_profile._last
    is_closed = prof['is_closed'] if prof is not None else False

    err = np.sqrt((xa - x_seq) ** 2 + (ya - y_seq) ** 2)
    err_path = point_to_path_distance(xa, ya, path)

    fig = plt.figure(figsize=(18, 12))
    gs = fig.add_gridspec(3, 3, hspace=0.40, wspace=0.35)

    ax1 = fig.add_subplot(gs[0, 0])
    s_dense = np.linspace(0, path.total, 800)
    xp = [path.eval(si)[0] for si in s_dense]
    yp = [path.eval(si)[1] for si in s_dense]
    ax1.plot(xp, yp, '-', color='gray', lw=0.8, alpha=0.6)
    for si in np.linspace(0, path.total, 20):
        xi, yi, txi, tyi = path.eval(si)
        ax1.arrow(xi, yi, txi * 6, tyi * 6,
                  head_width=2, head_length=2, fc='red', ec='red', alpha=0.5)
    ax1.set_aspect('equal')
    ax1.set_title(f'Step 2: Arc-length Parameterization\n'
                  f'Total={path.total:.0f}mm, {"CLOSED" if is_closed else "OPEN"}',
                  fontsize=10)
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(s_samples, v_curvature_only, '--', color='orange', lw=1.5,
             alpha=0.8, label='curvature+accel (forward)')
    ax2.plot(s_samples, v_limits, '-', color='#2196F3', lw=2.5,
             label='final (backward)')
    decel_mask = v_limits < v_curvature_only - 1.0
    ax2.fill_between(s_samples, v_limits, v_curvature_only,
                     where=decel_mask, color='red', alpha=0.2)
    ax2.axhline(v_seq.max() if len(v_seq) > 0 else 500, color='gray',
                ls=':', lw=0.8)
    ax2.set_xlabel('Arc length s (mm)')
    ax2.set_ylabel('Speed limit (mm/s)')
    ax2.set_title(f'Step 3: Velocity Profile (arc-domain)\n'
                  f'v_max={v_seq.max():.0f}  amax={amax}', fontsize=10)
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs[0, 2])
    n_plt = 600
    s_plt = np.linspace(0, path.total, n_plt)
    x_plt, y_plt, v_plt = [], [], []
    for si in s_plt:
        xi, yi, _, _ = path.eval(si)
        x_plt.append(xi); y_plt.append(yi)
        v_plt.append(np.interp(si, s_samples, v_limits))
    points = np.array([x_plt, y_plt]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    from matplotlib.collections import LineCollection
    norm = plt.Normalize(0, v_seq.max() if len(v_seq) > 0 else 500)
    lc = LineCollection(segments, cmap='RdYlGn', norm=norm, linewidth=4)
    lc.set_array(np.array(v_plt[:-1]))
    ax3.add_collection(lc)
    plt.colorbar(lc, ax=ax3, shrink=0.8, label='Speed limit (mm/s)')
    ax3.plot(x_plt[0], y_plt[0], 'o', color='green', ms=8, zorder=5)
    ax3.plot(x_plt[-1], y_plt[-1], 's', color='red', ms=8, zorder=5)
    ax3.set_aspect('equal')
    ax3.set_title('Speed Limit Heatmap', fontsize=10)
    ax3.grid(True, alpha=0.2)

    ax4 = fig.add_subplot(gs[1, 0])
    vlim_t = np.interp(np.linspace(0, path.total, len(t_seq)), s_samples, v_limits)
    ax4.plot(t_seq, vlim_t, '--', color='orange', lw=1, alpha=0.7, label='v_lim(s(t))')
    ax4.plot(t_seq, v_seq, '-', color='#2196F3', lw=1.5, label='v(t)')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Speed (mm/s)')
    ax4.set_title(f'Step 4: Combined Speed vs Limit\n'
                  f'Total time={t_seq[-1]:.2f}s', fontsize=10)
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)

    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(t_seq, vx_seq, '-', lw=1.2, label='vx_target → X')
    ax5.plot(t_seq, vy_seq, '-', lw=1.2, label='vy_target → Y')
    if vx_ref is not None:
        ax5.plot(t_seq, vx_ref, '--', lw=1.0, alpha=0.7, color='#2196F3',
                 label='vx_ref (S-curve out)')
        ax5.plot(t_seq, vy_ref, '--', lw=1.0, alpha=0.7, color='#4CAF50',
                 label='vy_ref (S-curve out)')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Target velocity (mm/s)')
    ax5.set_title('★ Output: XY Target Velocities ★', fontsize=10)
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    ax6 = fig.add_subplot(gs[1, 2])
    ax6.plot(xp, yp, '--', color='gray', lw=0.8, alpha=0.5, label='Target')
    ax6.plot(xa, ya, '-', color='#2196F3', lw=1.0, label='Simulated actual')
    ax6.plot(xa[0], ya[0], 'go', ms=4)
    ax6.plot(xa[-1], ya[-1], 'ro', ms=4)
    ax6.set_aspect('equal')
    ax6.set_title(f'Verification: S-curve Simulation\n'
                  f'max same-time err={np.max(err):.3f}  '
                  f'max dist-to-path={np.max(err_path):.3f}mm', fontsize=10)
    ax6.legend(fontsize=7)
    ax6.grid(True, alpha=0.3)

    ax7 = fig.add_subplot(gs[2, 0])
    ax7.plot(t_seq, vx_seq, '-', color='#2196F3', lw=1.0, label='vx_target')
    if vx_ref is not None:
        ax7.plot(t_seq, vx_ref, '--', color='#FF9800', lw=1.0, label='vx_ref')
        ax7.plot(t_seq, vx_act, '-', color='#4CAF50', lw=0.8, alpha=0.7,
                 label='vx_act (sim)')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('X Velocity (mm/s)')
    ax7.set_title('X Axis: target → v_ref → actual', fontsize=10)
    ax7.legend(fontsize=7)
    ax7.grid(True, alpha=0.3)

    ax8 = fig.add_subplot(gs[2, 1])
    ax8.plot(t_seq, vy_seq, '-', color='#4CAF50', lw=1.0, label='vy_target')
    if vy_ref is not None:
        ax8.plot(t_seq, vy_ref, '--', color='#FF9800', lw=1.0, label='vy_ref')
        ax8.plot(t_seq, vy_act, '-', color='#2196F3', lw=0.8, alpha=0.7,
                 label='vy_act (sim)')
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('Y Velocity (mm/s)')
    ax8.set_title('Y Axis: target → v_ref → actual', fontsize=10)
    ax8.legend(fontsize=7)
    ax8.grid(True, alpha=0.3)

    ax9 = fig.add_subplot(gs[2, 2])
    ax9.plot(t_seq, err, '-', color='red', lw=0.8,
             label=f'same-time (max={np.max(err):.2f})')
    ax9.plot(t_seq, err_path, '-', color='#2196F3', lw=1.2,
             label=f'dist-to-path (max={np.max(err_path):.2f})')
    ax9.set_xlabel('Time (s)')
    ax9.set_ylabel('Error (mm)')
    ax9.set_title('Contour Error\nred=same-time  blue=true (to path)', fontsize=10)
    ax9.legend(fontsize=7)
    ax9.grid(True, alpha=0.3)

    fig.suptitle(f'Contour Following Pipeline v2 — {name}  (amax={amax} mm/s²)',
                 fontsize=14, fontweight='bold')
    plt.savefig(f'pipeline_{name}.png', dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  [Result] {name}: time={t_seq[-1]:.2f}s  '
          f'same-time err(max/mean)={np.max(err):.3f}/{np.mean(err):.3f}mm  '
          f'true contour err(max/mean)={np.max(err_path):.3f}/{np.mean(err_path):.3f}mm')




def export_csv(name, t_seq, vx_seq, vy_seq, x_seq, y_seq):
    filename = f'target_velocities_{name}.csv'
    header = 'time_s,arc_length_mm,combined_v_mm_s,vx_target_mm_s,vy_target_mm_s,x_mm,y_mm'
    data = np.column_stack([t_seq, np.cumsum(np.concatenate([[0],
                            np.sqrt(np.diff(x_seq)**2 + np.diff(y_seq)**2)])),
                            np.sqrt(vx_seq**2 + vy_seq**2),
                            vx_seq, vy_seq, x_seq, y_seq])
    np.savetxt(filename, data, delimiter=',', header=header, fmt='%.4f')
    print(f'  [Export] → {filename} ({len(t_seq)} frames)')



if __name__ == '__main__':
    do_export = '--export' in sys.argv
    step_filter = None
    if '--step' in sys.argv:
        idx = sys.argv.index('--step')
        step_filter = int(sys.argv[idx + 1])
    seed = None
    if '--seed' in sys.argv:
        seed = int(sys.argv[sys.argv.index('--seed') + 1])

    # 运动参数 — 改你电机的实际值
    V_MAX = 500   # 直道最高速度 (mm/s)
    AMAX  = 1000  # 电机最大加速度 (mm/s²) ← 和 motor_controller.py 一致
    DT    = 0.001 # 控制周期 (s)
    TAU   = 0.005 # 电机时间常数
    KP    = 5.0   # P 增益

    if '--input' in sys.argv:
        in_path = sys.argv[sys.argv.index('--input') + 1]
        points = read_points_csv(in_path)
        name = os.path.splitext(os.path.basename(in_path))[0]
        src = in_path
    else:
        points = random_contour_points(seed=seed)
        name = 'Random'
        src = f'seed={seed if seed is not None else "random"}'
    print(f"\n{'='*60}")
    print(f"  {name}  ({len(points)} points)  {src}")
    print(f"{'='*60}")

    path = build_path(points)
    print(f"  [Step 2] {len(path.segs)} segments, total={path.total:.1f}mm")
    if step_filter == 2:
        sys.exit(0)

    s_samp, v_lims, v_curv = build_velocity_profile(path, V_MAX, AMAX)
    prof = build_velocity_profile._last
    print(f"  [Step 3] {'CLOSED' if prof['is_closed'] else 'OPEN'}  "
          f"κ_eff max: {prof['kappa'].max():.3f}  "
          f"final limit min: {np.min(v_lims):.0f} mm/s")
    if step_filter == 3:
        sys.exit(0)

    t_seq, v_seq, vx_seq, vy_seq, x_seq, y_seq = plan_motion(
        path, s_samp, v_lims, V_MAX, AMAX, DT)
    print(f"  [Step 4] {len(t_seq)} frames, {t_seq[-1]:.2f}s, "
          f"v_max={np.max(v_seq):.0f} mm/s")
    if step_filter == 4:
        sys.exit(0)

    if '--simple' in sys.argv:
        xa, ya = simulate_simple_motor(vx_seq, vy_seq, x_seq, y_seq, DT, TAU)
        vx_ref = vy_ref = vx_act = vy_act = None
    else:
        xa, ya, vx_ref, vy_ref, vx_act, vy_act = simulate_with_motor_controller(
            vx_seq, vy_seq, x_seq, y_seq, DT, AMAX, TAU, kp=KP)

    plot_results(path, s_samp, v_lims, v_curv,
                 t_seq, v_seq, vx_seq, vy_seq, x_seq, y_seq,
                 xa, ya, AMAX, name,
                 vx_ref, vy_ref, vx_act, vy_act)

    if do_export:
        export_csv(name, t_seq, vx_seq, vy_seq, x_seq, y_seq)

    print(f"\n{'='*60}")
    print(f"  Done → pipeline_{name}.png")
    print(f"  控制器 (每 {DT*1000:.0f}ms):")
    print(f"    v_ref_x, cmd_x = mc_x.update(vx_target, encoder_x())")
    print(f"    v_ref_y, cmd_y = mc_y.update(vy_target, encoder_y())")
    print(f"{'='*60}")
