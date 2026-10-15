"""
======================================================================
 单轴 S 曲线电机运动控制器 — 速度 / 位置 / 力矩 三种输入
 七次多项式 S 曲线平滑 + 速度/加速度前馈 + PID 反馈
======================================================================

 功能: 任意目标速度/位置/力矩输入 → 七次 S 曲线平滑 (物理可行的加减速)
       → 电机控制量。双轴轮廓跟随时每轴一个实例, 多轴同理各一个。

【三种用法 (都是每个控制周期调用一次)】
   1. 速度控制: update(target_speed, actual_velocity)
       任意目标速度 → 七次 S 曲线平滑 → 前馈 + PID → cmd
   2. 位置控制: Moveposition 应用层, 输入目标位置, 内部复用 mc.update()
       mp = Moveposition(mc);   v_ref, cmd = mp(target_position, current_position)
       实现 = 位置-速度映射 (控制器本身不改, 纯速度控制), 每帧从实测位移重算:
          v_tgt = sign(d)·min(0.85·v_max, √(amax·|d|/2.1875)),  d = 剩余位移
       √(amax·|d|/2.1875) = 剩余 |d| 能承载的最高速度 (保守反解: 七次曲线
       单程刹停距离 = 1.09375·V²/amax; 映射取往返距离 2.1875·V²/amax
       → 留 2 倍余量, 配合实测 d 保证任何跟踪滞后都刹得住),
       由实测 d 驱动 → 自校正的刹停曲线:
          大位移自动巡航 0.85·v_max, 小位移自动降速 (三角剖),
          目标突变每帧重算, 加减速由七次曲线状态机平滑推进 (目标变化率
          ≤ amax·dt → 续接不重启, 无需阶梯化)
       关键: 减速曲线直接挂在实测 d 上, 电机跟踪滞后/负载扰动不影响终点
       —— d 收敛到 pos_tol 时 v_tgt 同步收敛到 0, 第一次就到位,
       无"表走完未到位"的兜底重规划, 无重新加速尖峰
       到位: 死区+迟滞 (|d| ≤ pos_tol 锁定 0, 超 pos_tol+pos_hys 才解锁)
             且目标速度低通 (v_tgt_tau) → 抗编码器量化噪声引起的 d 的
             sign 高频翻转, 稳定停位, 停稳后 mp._arrived = True
   3. 力矩控制: TorqueControl 应用层, 输入目标力矩, 输出力矩指令
       (复用同一 mc 的 S曲线状态机: 平滑/限幅; PID 不参与 — 力矩/速度无量纲)
       tc = TorqueControl(mc);   torque_ref, cmd = tc(目标力矩[, 实测速度])
       mc.v_max = 力矩限幅, mc.amax = 力矩爬升率 (单帧 ≤ amax·dt);
       可选 kd_v 速度阻尼 — 详见类 docstring。
       力矩环表现: cmd = 恒力矩 → 电机按牛顿定律响应 (恒力矩 → 匀加速),
       由驱动器/电机动力学决定, 控制器只保证力矩指令平滑可达。

【用法示例】
   from motor_controller import MotorController

   mc = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000, v_max=500.0)

   # 速度控制: 每个控制周期调用一次 (周期 = dt):
   v_ref, cmd = mc.update(target_speed=100.0, actual_velocity=98.3)   # 有编码器闭环
   v_ref, cmd = mc.update(target_speed=100.0)                          # 无编码器开环

   # 位置控制: 每个控制周期调用一次 (独立应用层, 复用 mc.update):
   mp = Moveposition(mc)                        # 位置应用实例
   v_ref, cmd = mp(100.0, encoder_pos)          # 走到 100mm 停住
   if mp._arrived: ...                          # 到位查询

   # 力矩控制: 每个控制周期调用一次 (与位置模式同构, 复用同一个 mc):
   tc = TorqueControl(mc, kd_v=0.5)
   torque_ref, cmd = tc(0.8)                   # 力矩指令 (可选附加阻尼, 见类 docstring)

   # 换目标 / 急停后复位内部状态:
   mc.reset(); mp.reset(); tc.reset()

【返回】
   v_ref : S曲线平滑后的参考速度 (mm/s)
   cmd   : 电机控制量 = 前馈 + PID 输出 (限幅在 out_max 内)

【参数】MotorController(kp, ki, kd, dt, amax, i_max, out_max, kff_vel, kff_acc, smooth_tau, v_max)
   kp, ki, kd   : PID 增益
   dt           : 控制周期 (s) ← 必须与你的控制循环周期一致
   amax         : 最大加速度 (mm/s²) ← 硬约束, 任何工况单帧变化 ≤ amax·dt
   i_max        : 积分限幅 (默认 1.0)
   out_max      : PID 输出限幅 (默认 1.0)
   kff_vel      : 速度前馈系数 (默认 1.0)
   kff_acc      : 加速度前馈系数 (默认 0.0, 已知电机时间常数后可开)
   smooth_tau   : 输出一阶低通时间常数 (s, 默认 0 = 关闭; 指令太顿再打开)
   v_max        : 速度上限 (mm/s, 默认 500.0); 目标速度钳制在 ±v_max 内,
                  位置控制匀速速度 Ve = 0.85·v_max

【换电机只需改】kp/ki/kd + dt + amax + v_max (+ 限幅)
   注意 amax 要与生成目标速度的上游规划器一致, 否则规划出的速度电机跟不上。

【自测】python motor_controller.py — 跑速度自测 + 位置自测 + 力矩自测 + 工作循环仿真
   出图: motor_controller_sim.png (一张总图: 左列速度仿真, 右列位置工作循环仿真)
   工作循环仿真带随机噪声 (每次运行略不同), 固定复现: random.seed(42)
======================================================================
"""

class Moveposition:
    """
    位置控制应用层: 输入目标位置, 电机运动到位后停下。
    只复用速度控制器 mc.update() (不改控制器内部), 是"上层应用"式实现。

    位置-速度映射 (每帧从实测位移重算, 自校正的刹停曲线):
        v_tgt = sign(d)·min(0.85·v_max, √(Amax·|d|/2.1875)),  d = 剩余位移
        √(Amax·|d|/2.1875) = 剩余 |d| 能承载的最高速度
        (七次曲线单程刹停距离 = 1.09375·V²/Amax ≈ 0.5·V·T;
         映射取的 2.1875·V²/Amax 是"加速+刹停"往返距离 → 保守 2 倍余量)

    为什么"第一次就到位" (无兜底重规划):
        减速曲线直接挂在实测 d 上 —— d 大时映射值 ≥ 巡航 → 自动巡航;
        d 缩小时映射值沿 √ 曲线下降 → 电机沿"恰好刹停"的曲线减速;
        电机跟踪滞后/负载扰动只影响沿曲线的前后位置, 不影响终点:
        d → pos_tol 时 v_tgt 同步 → 0, 参考速度第一次就停在目标上。
        对比"时间表按理想电机规划减速距离": 真实电机滞后导致刹停后残余
        ~0.7mm → 必须兜底重规划 → 0→±20→0 重新加速尖峰。映射版从机制上消除。

    加速/减速平滑: 目标速度每帧变化率 ≤ Amax·dt (√ 映射曲线实际斜率 =
    Amax/4.375 ≈ 0.23·Amax, 远低于硬限 → 减速过程本就柔和),
    七次曲线状态机续接 (理想下一帧反解) 平滑跟进, 不重启不卡死。
    小位移自动三角剖: 映射值自动低于巡航, 山峰形剖面, 无需专门规划。
    目标中途修改: 每帧重算映射, 立即响应, 无需停稳。

    【抗噪声设计 — 编码器量化 + 测量噪声下的稳定停位】
    现象: 接近目标时 |d| 与编码器本底噪声同量级, d 的符号被噪声带得
    高频翻转 → v_tgt = ±小速度反复横跳; 每次翻转 S 曲线状态机都被强切
    (tau 归零重算), a_ref 在 ±Amax 间来回猛切 → 高频正反向大电流。
    对策 (两层, 只在停位区附近生效, 不影响大位移段):
      1. 死区 + 迟滞锁存: |d| ≤ pos_tol 判为到位并锁存目标速度 0;
         只有 |d| > pos_tol + pos_hys 才解锁重算。读数贴着死区边界抖动
         也只有一次"滑入", 不会 0 ↔ ±v 高频往返 (边界极限环)。
      2. 目标速度一阶低通 (v_tgt_tau): 小 d 时 √ 曲线输出仅 ±几 mm/s,
         低通后 sign 翻转的残幅 ≈ 噪声含量的几个百分点 → 不给 S 曲线
         喂"脏"目标。仅衰减不进位, 大位移巡航/刹车段不受影响。
      3. 锁存后目标直接给 0 (跳过低通): 停位不拖尾、无低通缓坡过冲。

    用法:
        mc = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000,
                             v_max=300.0)   # 速度上限, 位置匀速 = 0.85·v_max
        mp = Moveposition(mc)               # 位置应用实例
        pos = 0.0
        while running:                      # 每个控制周期
            v_ref, cmd = mp(100.0, pos)     # 输入目标位置 (mm), 返回同 update()
            pos, vel = motor.step(cmd, dt)  # 编码器读数
            if mp._arrived:                 # 到位并停稳 (可查询)
                ...
    """

    def __init__(self, mc, pos_tol=0.05, pos_hys=None, v_tgt_tau=0.02):
        """
        参数:
            mc        : MotorController 实例 (用其 update/v_max/amax/dt)
            pos_tol   : 停位误差容差 (mm, 默认 0.05). |d| ≤ pos_tol 判为进入死区
            pos_hys   : 死区迟滞裕量 (mm, 默认 = pos_tol). 只有
                        |d| > pos_tol + pos_hys 才解锁, 防边界抖动
            v_tgt_tau : 目标速度一阶低通时间常数 (s, 默认 0.02; 0 = 关闭)
        """
        self.mc = mc
        self.pos_tol = pos_tol
        self.pos_hys = pos_tol if pos_hys is None else pos_hys
        self.v_tgt_tau = v_tgt_tau
        self._v_ref = 0.0           # 上一帧 update 返回的 v_ref
        self._phase = 'DONE'        # 当前阶段: CRUISE (巡航) / BRAKE (刹停) / DONE
        self._arrived = False       # True = 已到位并停稳
        self._stopped = False       # True = 已在死区内锁存停位 (防边界抖动)
        self._v_tgt_s = 0.0         # v_tgt 低通滤波值 (锁存时冻结)

    def __call__(self, target_position, current_position, actual_velocity=None):
        """
        每个控制周期调用一次: 输入目标位置, 返回 (v_ref, cmd) 同 mc.update()。

        位置-速度映射每帧重算: 目标速度只有 0 / 巡航 / √(Amax·|d|/2.1875),
        由实测 d 驱动 → 自校正, 第一次就到位, 无兜底重规划。
        死区锁存 + 目标低通 (抗噪声): 见类 docstring【抗噪声设计】。
        """
        mc = self.mc
        d = target_position - current_position
        if self._stopped and abs(d) <= self.pos_tol + self.pos_hys:
            # 死区内锁存: 不重算目标 (防边界抖动), 直接 0, 等 v_ref 衰减
            v_tgt = 0.0
            self._phase = 'DONE'
            self._arrived = abs(self._v_ref) < 1e-6   # 停稳才算到位
        else:
            if self._stopped:
                self._stopped = False       # |d| 明确超出死区 → 解锁
            if abs(d) <= self.pos_tol:
                self._stopped = True        # 首次 |d| ≤ pos_tol → 进入并锁存
                v_tgt = 0.0
                self._phase = 'DONE'
                self._arrived = abs(self._v_ref) < 1e-6
            else:
                self._arrived = False
                v_cruise = 0.85 * mc.v_max
                v_brake = math.sqrt(mc.amax * abs(d) / 2.1875)  # 剩余 |d| 的刹停上限
                if v_brake >= v_cruise:
                    v_tgt = math.copysign(v_cruise, d)
                    self._phase = 'CRUISE'
                else:
                    v_tgt = math.copysign(v_brake, d)
                    self._phase = 'BRAKE'
                # 目标速度一阶低通: sign 抖动时 |v_tgt| 仅 ±几 mm/s, 低通后
                # 残幅≈噪声含量几个百分点, S 曲线不再被高频翻转强切
                alpha = 1.0 if self.v_tgt_tau <= 0 else \
                    mc.dt / (self.v_tgt_tau + mc.dt)
                self._v_tgt_s += alpha * (v_tgt - self._v_tgt_s)
                v_tgt = self._v_tgt_s
        v_ref, cmd = mc.update(v_tgt, actual_velocity)
        self._v_ref = v_ref
        return v_ref, cmd

    def reset(self):
        """复位锁存/滤波/到位状态 (重新批次时调用; 需同时复位 S 曲线则可调 mc.reset())"""
        self._stopped = False
        self._v_tgt_s = 0.0
        self._arrived = False
        self._v_ref = 0.0
        self._phase = 'DONE'


class TorqueControl:
    """
    力矩指令应用层: 目标力矩 → S曲线平滑 → 力矩指令
    (与 Moveposition 同构: 薄应用层 + 复用同一个 mc 实例, 输入换成力矩)

    复用性: 内部就是 mc.update(target_torque, None) — 复用同一 S曲线状态机
    (平滑/限幅/爬升率硬约束), 状态机本身对通道内容无感:
        mc.v_max  = 力矩限幅 (±): 目标力矩被钳制在 ±v_max 内
        mc.amax   = 力矩爬升率:   任意工况单帧力矩变化 ≤ amax·dt (硬约束)
    PID 不参与: 力矩 vs 速度无量纲可比, 闭环会把力矩环拉回"速度=力矩"的
    伺服行为 — 而力矩环的表现应由电机动力学决定 (恒力矩 → 匀加速)。

    参数语义始终不变: mc.v_max = 电机最大速度, mc.amax = 电机最大加速度,
    都是物理极限, 任何模式不突破。S 曲线状态机在力矩通道只做【信号平滑】
    (它平滑的对象是力矩, 不是速度); 力矩 → 速度由电机动力学决定, 物理
    极限由机器模型硬执行:
      TorqueControl.torque_max = 力矩信号钳幅 (机器额定力矩, 默认 mc.v_max 兜底)
      SimulatedMotor.v_max     = 电机限速 (到达 ±v_max 即钳速, 不可越线)
      SimulatedMotor.a_max     = 电机最大加速度 (请求加速度超出即截断)
    所以力矩指令再大, 电机也只能按自己的 v_max / amax 极限运行。

    返回:
        torque_ref : S曲线平滑后的参考力矩
        cmd        : 力矩指令 = torque_ref (可选速度阻尼)

    可选速度阻尼 (kd_v > 0): cmd = torque_ref − kd_v·vel (钳制在 ±mc.v_max)
        力矩驱动低阻负载时提供最基本的稳定性 (经典力控 F = F_cmd − kd·v);
        阻尼 ≠ 速度跟踪 (要跟踪速度/位置请用速度/位置模式)。
    actual_velocity 传入后仅用于阻尼, 不触发 PID。

    用法:
        mc = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001,
                             amax=1000, v_max=500)   # 与速度/位置模式共用
        tc = TorqueControl(mc, kd_v=0.0)
        # 每个控制周期一次:
        torque_ref, cmd = tc(0.8)                     # 纯力矩指令
        torque_ref, cmd = tc(0.8, actual_velocity)    # 附加速度阻尼
        tc.reset()                                    # 复位当前目标轨迹状态

    换执行机构: 重设 mc.v_max / mc.amax (电机物理极限) 与 torque_max (额定力矩);
    dt 必须与控制循环一致。
    """

    def __init__(self, mc, kd_v=0.0, torque_max=None):
        """
        参数:
            mc         : MotorController 实例 (与速度/位置模式共用的是同一个)
            kd_v       : 速度阻尼系数 (力矩单位/速度单位, 默认 0 = 关)
            torque_max : 力矩信号钳幅 = 机器额定力矩 (None = 沿用 mc.v_max 兜底);
                         注意这不是"速度限幅" — 速度/加速度极限在机器模型侧
                         (SimulatedMotor.v_max / a_max)
        """
        self.mc = mc
        self.kd_v = kd_v
        self.torque_max = torque_max
        self._torque_ref = 0.0      # 上一帧 S 曲线力矩输出 (调试/复位用)

    def __call__(self, target_torque, actual_velocity=None):
        """
        每个控制周期调用一次: 输入目标力矩, 返回 (torque_ref, cmd)。

        复用 mc.update 的 S 曲线状态机 (开环平滑, PID 不参与);
        actual_velocity 可选, 提供时仅用于 kd_v 速度阻尼。
        力矩目标先按机器额定力矩钳幅 (torque_max), 阻尼输出同幅值钳制 —
        物理极限 (v_max 限速 / amax 过载) 在机器模型侧硬执行。
        """
        lim = self.torque_max if self.torque_max is not None else self.mc.v_max
        if target_torque > lim:
            target_torque = lim
        elif target_torque < -lim:
            target_torque = -lim
        torque_ref, _ = self.mc.update(target_torque, None)
        cmd = torque_ref
        if actual_velocity is not None and self.kd_v != 0.0:
            cmd = torque_ref - self.kd_v * actual_velocity
        # 输出恒钳幅 ≤ ±lim (额定力矩): 跨模式续接等情况下 τ_ref 可能暂超
        # 新目标限 (S曲线状态里带的旧值), 指令必须始终 ≤ 机器额定力矩
        if cmd > lim:
            cmd = lim
        elif cmd < -lim:
            cmd = -lim
        self._torque_ref = torque_ref
        return torque_ref, cmd

    def reset(self):
        """复位力矩轨迹状态 (换目标/急停后调用; 需同时复位 mc 时再调 mc.reset())"""
        self._torque_ref = 0.0


def _h(tau):
    """
    h(τ) = 35τ⁴ − 84τ⁵ + 70τ⁶ − 20τ⁷,   τ ∈ [0, 1]

    h(0)=0,    h(1)=1
    h'(0)=h'(1)=0     (起始/终止加速度=0)
    h''(0)=h''(1)=0   (起始/终止 jerk=0)
    h'''(0)=h'''(1)=0 (起始/终止 snap=0)
    """
    t2 = tau * tau
    t4 = t2 * t2
    return 35.0 * t4 - 84.0 * t4 * tau + 70.0 * t4 * t2 - 20.0 * t4 * t2 * tau


def _dh_dtau(tau):
    """h(τ) 的导数 dh/dτ = 140τ³ − 420τ⁴ + 420τ⁵ − 140τ⁶"""
    t2 = tau * tau
    t3 = t2 * tau
    t4 = t2 * t2
    return 140.0 * t3 - 420.0 * t4 + 420.0 * t4 * tau - 140.0 * t4 * t2


def _find_tau(h_val, tol=1e-7):
    """
    七次 S 曲线反函数: 已知 h(τ) 值, 求 τ ∈ [0, 1]

    h(τ) 在 [0,1] 上单调递增, 可用二分法快速求解。
    """
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) * 0.5
        if _h(mid) < h_val:
            lo = mid
        else:
            hi = mid
    return (lo + hi) * 0.5



class MotorController:
    """
    单轴 S 曲线电机速度控制器

    用法:
        mc = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000)

        # 有编码器 → 闭环 (S曲线 + 前馈 + PID)
        v_ref, cmd = mc.update(target_speed=100.0, actual_velocity=98.3)

        # 无编码器 → 开环 (S曲线 + 前馈, 速度从 S 曲线内部估算)
        v_ref, cmd = mc.update(target_speed=100.0)
    """

    def __init__(self, kp=0.5, ki=0.02, kd=0.01,
                 dt=0.001, amax=1000.0,
                 i_max=1.0, out_max=1.0,
                 kff_vel=1.0, kff_acc=0.0,
                 smooth_tau=0.0,
                 v_max=500.0):
        """
        参数:
            kp, ki, kd : PID 增益
            dt         : 控制周期 (s)
            amax       : 最大加速度 (mm/s²) ← 硬约束, 任何工况都不超
            i_max      : 积分限幅
            out_max    : PID 总输出限幅
            kff_vel    : 速度前馈系数 (默认 1.0)
            kff_acc    : 加速度前馈系数 (默认 0.0, 已知 tau 后可启用)
            smooth_tau : 输出一阶低通时间常数 (s, 默认 0 = 关闭, 需要更钝的指令时再打开)
            v_max      : 速度上限 (mm/s, 默认 500.0); 目标速度被钳制在 ±v_max 内,
                         位置应用 (Moveposition) 的匀速速度取 0.85·v_max
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.amax = amax
        self.i_max = i_max
        self.out_max = out_max
        self.kff_vel = kff_vel
        self.kff_acc = kff_acc
        self.alpha = dt / (smooth_tau + dt)
        self.v_max = v_max


        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_deriv = 0.0

        self._v_ref_smooth = 0.0

        self._Vi = 0.0
        self._Ve = 0.0
        self._T = 0.0
        self._t = 0.0
        self._active = False


    def _pid_control(self, error):
        """PID 控制器 (比例 + 梯形积分限幅 + 微分低通)"""
        dt = self.dt
        if dt < 1e-9:
            return 0.0

        p_term = self.kp * error

        self._integral += 0.5 * self.ki * dt * (error + self._prev_error)
        if self._integral > self.i_max:
            self._integral = self.i_max
        if self._integral < -self.i_max:
            self._integral = -self.i_max

        deriv_raw = (error - self._prev_error) / dt
        tau_d = dt * 7.0
        alpha = dt / (tau_d + dt)
        deriv_filtered = alpha * deriv_raw + (1.0 - alpha) * self._prev_deriv
        self._prev_deriv = deriv_filtered
        d_term = self.kd * deriv_filtered

        out = p_term + self._integral + d_term

        if out > self.out_max:
            out = self.out_max
        if out < -self.out_max:
            out = -self.out_max

        self._prev_error = error
        return out


    def _get_v_ref(self):
        """当前 S 曲线上的瞬时速度 v_ref(τ) = (Ve-Vi)×h(τ) + Vi"""
        if not self._active:
            return self._Ve
        tau = self._t / self._T
        return (self._Ve - self._Vi) * _h(tau) + self._Vi

    def _get_a_ref(self):
        """当前 S 曲线上的瞬时加速度 a_ref = dv_ref/dt"""
        if not self._active or abs(self._T) < 1e-9:
            return 0.0
        tau = self._t / self._T
        dh = _dh_dtau(tau)
        return ((self._Ve - self._Vi) / self._T) * dh

    def _scurve_plan(self, Ve):
        if self._active:
            tau_old = self._t / self._T if self._T > 1e-9 else 0.0
            v_now = (self._Ve - self._Vi) * _h(tau_old) + self._Vi
        else:
            v_now = self._Ve

        dVe = Ve - self._Ve
        dV = Ve - self._Vi
        a_lim = self.amax * self.dt
        if self._active and abs(dV) > 1e-9:
            if abs(dVe) > 1e-9:
                # 目标在动 → 按速度误差收敛 (≤ amax·dt): 目标以 ≤amax 变化时
                # v_next=v_tgt 平滑续接; 目标跳变/差距大时以 amax 追近收敛。
                # 不能用目标增量 (ΔVe) 追: 目标缓降而速度差距大时,
                # 每帧只走目标的速率 → 差距永不收敛 → 冲过目标 (到位尖峰)。
                v_next = v_now + min(max(Ve - v_now, -a_lim), a_lim)
            else:
                v_next = v_now + self._get_a_ref() * self.dt
            h_next = (v_next - self._Vi) / dV
            if 0.0 <= h_next <= 1.0:
                tau_end = _find_tau(h_next)
                T_new = 2.1875 * abs(dV) / self.amax
                tau_start = tau_end - self.dt / T_new
                if tau_start < 0.0:
                    self._Vi = v_now
                    tau_start = 0.0
            else:
                self._Vi = v_now
                tau_start = 0.0
        else:
            self._Vi = v_now
            tau_start = 0.0

        dV = Ve - self._Vi
        if abs(dV) < 1e-9:
            self._active = False
            self._T = 0.0
            self._t = 0.0
            return Ve
        self._Ve = Ve
        self._T = 2.1875 * abs(dV) / self.amax
        self._t = tau_start * self._T
        self._active = True

        self._t += self.dt
        if self._t >= self._T:
            self._active = False
            self._t = self._T
        return self._get_v_ref()


    def update(self, target_speed, actual_velocity=None):
        """
        每个控制周期调用一次

        参数:
            target_speed    : 目标速度 (mm/s 或任意单位)
            actual_velocity : 编码器实测速度 (同单位), 可选。
                              有编码器 → 闭环: S曲线 + 前馈 + PID
                              无编码器 → 开环: S曲线 + 前馈
                              (内部用 S 曲线自身的 v_ref 作为速度估计)

        返回:
            v_ref : S 曲线平滑后的参考速度
            cmd   : 电机控制量 (前馈 + PID 或纯前馈)
        """
        # 目标速度限幅: 任何目标都不超过 ±v_max
        if target_speed > self.v_max:
            target_speed = self.v_max
        elif target_speed < -self.v_max:
            target_speed = -self.v_max

        if actual_velocity is None:
            actual_velocity = self._get_v_ref()
            use_feedback = False
        else:
            use_feedback = True

        v_ref = self._scurve_plan(target_speed)
        a_ref = self._get_a_ref()

        self._v_ref_smooth += self.alpha * (v_ref - self._v_ref_smooth)
        v_out = self._v_ref_smooth

        u_ff = self.kff_vel * v_out + self.kff_acc * a_ref

        if use_feedback:
            error = v_out - actual_velocity
            pid_out = self._pid_control(error)
        else:
            pid_out = 0.0

        cmd = pid_out + u_ff
        return v_out, cmd

    def reset(self):
        """复位所有内部状态（换目标或急停后调用）"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_deriv = 0.0
        self._Vi = 0.0
        self._Ve = 0.0
        self._T = 0.0
        self._t = 0.0
        self._active = False
        self._v_ref_smooth = 0.0



import math
import random


class SimulatedMotor:
    """
    逼真的单轴电机仿真模型 (用于替代 simple first-order 模型)

    物理建模:
      位置 x, 速度 v, 加速度 a = (u - F_friction(v)) / m
      其中 u 是控制器输出 (归一化到 [-1,1]), m 是等效质量

    主要非理想因素:
      1. Coulomb + viscous 摩擦力 (静止时静摩擦 > 动摩擦)
      2. 执行器饱和 (cmd 限幅到 out_sat)
      3. 编码器量化噪声 (resolution) + 高斯白噪声
      4. 可选: 负载扰动 (step disturbance)
      5. 可选: 硬件超速保护 (v_max, 驱动器限速钳速)
      6. 可选: 机器最大加速度硬限 (a_max, 超加速请求截断)

    【用法】
        motor = SimulatedMotor(m=1.0, mu_c=0.3, mu_v=0.01,
                                resolution=0.01, noise_std=0.02)
        # 每个控制周期:
        pos_meas, vel_meas = motor.step(cmd, dt)
        # pos_meas, vel_meas 是带噪声的测量值 (模拟编码器读数)
    """

    def __init__(self, m=1.0, mu_c=0.3, mu_v=0.01,
                 resolution=0.01, noise_std=0.02,
                 out_sat=1.0, disturb_mag=0.0, disturb_t=0.0,
                 v_max=None, a_max=None, mu_s=None):
        """
        参数:
            m          : 等效质量/惯性 (mm/s² per unit cmd)
            mu_c       : Coulomb 摩擦系数 (占 cmd 的比例, 典型 0.1~0.5)
            mu_v       : Viscous 摩擦系数 (与速度成正比的阻尼)
            resolution : 编码器分辨率 (mm 或 mm/s)
            noise_std  : 高斯测量噪声标准差
            out_sat    : 执行器饱和限幅 (默认 1.0, 对应 cmd 限幅)
            disturb_mag: 阶跃负载扰动幅度 (cmd 单位), 0 = 无扰动
            disturb_t  : 扰动施加时刻 (s), 0 = 仿真开始就有
            v_max      : 电机最大速度 (mm/s, None = 不限制). 到达 ±v_max 后钳速,
                         不再加速上升 (驱动器超速保护) — 力矩模式 (无速度环) 必须
                         挂上, 否则恒力矩会让速度无上限增长 (v→(u−μ_c)/μ_v)
            a_max      : 电机最大加速度 (mm/s², None = 不限制). 请求加速度超出
                         ±a_max 时被机器截断 (电机极限!): 力矩环机器无内环,
                         控制器给多大力矩, 机器也只按自己的极限加速/减速 —
                         |a| 不可能超过 a_max
            mu_s       : 静摩擦系数 = 破静摩擦阈值 (None = 同 mu_c, 保持原行为).
                         静摩擦 > 动摩擦 才有真实 stick-slip: |u| ≤ mu_s 不起动,
                         破静摩擦 (|u|>mu_s) 后按动摩擦 mu_c 滑动

        注意: cmd 必须与 out_sat 同量纲 (默认语义 = 归一化 [-1,1] 驱动器)。
              MotorController 的 cmd 是 mm/s 量纲 (速度前馈 kff_vel=1 不归一化),
              直接配套使用时请把 out_sat 调到 ≥ 最大 cmd (如 500), 并重标定
              mu_c/mu_v (稳态 v ≈ (u−mu_c)/mu_v 应落在你的速度范围内)。
        """
        self.m = m
        self.mu_c = mu_c
        self.mu_v = mu_v
        self.resolution = resolution
        self.noise_std = noise_std
        self.out_sat = out_sat
        self.disturb_mag = disturb_mag
        self.disturb_t = disturb_t
        self.v_max = v_max
        self.a_max = a_max
        self.mu_s = self.mu_c if mu_s is None else mu_s   # 破静摩擦阈值 (≤ mu_c 等价旧行为)
        self._a_true = 0.0      # 本帧实际加速度 (硬限裁剪后, 供电机极限校验)

        self._pos = 0.0
        self._vel = 0.0
        self._vel_true = 0.0  # 真实速度 (不受噪声影响, 用于调试)

    def reset(self, pos0=0.0):
        self._pos = pos0
        self._vel = 0.0
        self._vel_true = 0.0
        self._a_true = 0.0

    def set_state(self, pos, vel):
        """
        续接位置/速度状态: 同一台物理电机以不同驱动器模式建模时
        (motor.py: v/p 模式 = 速度环伺服实例, t 模式 = 力矩环实例),
        模式切换时把旧实例的 (pos, vel) 交给新实例 — 否则新实例从 0
        出发, 位置/速度读数猛然归零 (假"丢位置")。
        """
        self._pos = pos
        self._vel = vel
        self._vel_true = vel
        self._a_true = 0.0

    def step(self, cmd, dt, t=None):
        """
        推进一个控制周期

        参数:
            cmd : 控制器输出 (归一化, 通常来自 MotorController.update)
            dt  : 控制周期 (s)
            t   : 当前仿真时间 (s), 用于施加阶跃扰动

        返回:
            pos_meas : 带量化+噪声的位置测量 (模拟编码器)
            vel_meas : 带量化+噪声的速度测量 (由位置差分得到)
        """
        # 执行器饱和
        u = max(-self.out_sat, min(self.out_sat, cmd))

        # 阶跃负载扰动
        if self.disturb_mag > 0 and t is not None and t >= self.disturb_t:
            u -= self.disturb_mag

        # 摩擦力模型
        v = self._vel
        if abs(v) < 1e-6:
            # 静止: 静摩擦抵抗运动趋势 (f 按 a=(u-f)/m 的阻力约定存储)
            #   |u| ≤ mu_s : 静摩擦恰好抵消指令 → a = 0 (静摩擦死区)
            #   |u| > mu_s : 破静摩擦, 只剩向下的动摩擦 sign(u)·mu_c
            #                (mu_s > mu_c → 真实 stick-slip: 起动摩擦更大)
            f_friction = math.copysign(self.mu_c, u) if abs(u) > self.mu_s else u
        else:
            # 运动: Coulomb + viscous (方向与速度相反)
            sign_v = 1.0 if v > 0 else -1.0
            f_friction = sign_v * self.mu_c + self.mu_v * abs(v) * sign_v

        # 加速度 = (控制力 - 摩擦力) / 质量
        a = (u - f_friction) / self.m

        # 电机极限: 加速度硬限 (a_max) — 请求的加速度被截断到 ±a_max。
        # 力矩环机器无内环 (a 直接是牛顿动力学结果), 极限必须由机器自己守:
        # 控制器给多大力矩, 机器也只能按自身极限加速/减速, |a| 绝不超 amax
        if self.a_max is not None:
            if a > self.a_max:
                a = self.a_max
            elif a < -self.a_max:
                a = -self.a_max
        self._a_true = a

        # RK1 积分 (Euler, 对仿真足够)
        self._vel_true = v + a * dt
        # 低速粘滞修正: Euler 单帧可能直接越过 v=0 (跳开 |v|<1e-6 判定),
        # 之后动摩擦分支 sign 每帧翻转 → 围绕 0 的 ±小速度极限环抖动
        # (t 模式指令 |u| ≤ 静摩擦时明显). 速度反号且指令 |u| ≤ μ_s
        # (静摩擦可扛住) → 粘住 v=0, 交回静止分支死区处理
        if self._vel_true * v < 0.0 and abs(u) <= self.mu_s:
            self._vel_true = 0.0
        self._pos += self._vel_true * dt

        # 速度钳制 (防止数值爆炸)
        if abs(self._vel_true) > 1e6:
            self._vel_true = 1e6 * math.copysign(1, self._vel_true)

        # 硬件超速保护 (驱动器限速): 到达 ±v_max 后钳速, 不再被加速 —
        # 力矩环无速度环兜底, 恒力矩会让速度无限增长 (v_ter=(u−μ_c)/μ_v),
        # v_max = 硬件规定上限 (None = 不限制, 保持原行为)
        if self.v_max is not None:
            if self._vel_true > self.v_max:
                self._vel_true = self.v_max
            elif self._vel_true < -self.v_max:
                self._vel_true = -self.v_max

        self._vel = self._vel_true

        # 测量噪声 + 量化
        pos_meas = self._quantize(self._pos + self._randn() * self.noise_std)
        vel_meas = self._quantize(self._vel_true + self._randn() * self.noise_std)

        return pos_meas, vel_meas

    def _randn(self):
        """高斯随机数 (Box-Muller, 避免 numpy 依赖)"""
        u1 = random.random()
        u2 = random.random()
        while u1 < 1e-10:
            u1 = random.random()
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)

    def _quantize(self, val):
        """编码器量化: 最接近 resolution 的整数倍"""
        if self.resolution <= 0:
            return val
        return round(val / self.resolution) * self.resolution


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    matplotlib.rcParams['axes.unicode_minus'] = False

    mc = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000)

    dt = 0.001
    T_total = 2.0
    steps = int(T_total / dt)

    import random, math
    random.seed()

    def continuous_target(t):
        """
        随机分段目标: 混合所有应用场景

        段类型 (随机抽取):
          STEP     : 大幅阶跃 (±50~250) - 触发续接/重启
          HOLD     : 保持当前速度 (长巡航)
          REVERSE  : 方向反转 (正负交替, 过零)
          MICRO    : 高频小幅微调 (目标小幅连续变化)
          STOP     : 减速到0 (终点停车)
        """
        if not hasattr(continuous_target, '_params'):
            n_segs = random.randint(8, 12)
            segs = []
            v = 0.0
            t_left = 2.5
            for k in range(n_segs):
                max_dur = max(0.15, t_left - 0.15 * (n_segs - k))
                dur = min(random.uniform(0.2, 0.6), max_dur)
                dur = max(dur, 0.15)
                seg_type = random.choice(['STEP', 'HOLD', 'REVERSE', 'MICRO'])
                if seg_type == 'STEP':
                    v += random.choice([-1, 1]) * random.uniform(50, 250)
                elif seg_type == 'HOLD':
                    pass
                elif seg_type == 'REVERSE':
                    v = random.choice([-1, 1]) * random.uniform(50, 300)
                elif seg_type == 'MICRO':
                    micro_segs = []
                    t_micro = 0.0
                    while t_micro < dur - 0.05:
                        m_dur = min(random.uniform(0.02, 0.08), dur - t_micro - 0.02)
                        m_dur = max(m_dur, 0.02)
                        m_v = v + random.uniform(-5, 5)
                        micro_segs.append((m_dur, m_v))
                        t_micro += m_dur
                    for m_dur, m_v in micro_segs:
                        segs.append((m_dur, m_v))
                    t_left -= dur
                    continue
                v = max(-300, min(300, v))
                segs.append((dur, v))
                t_left -= dur
            segs.append((max(t_left, 0.2), 0.0))
            bounds = [0.0]
            for dur, _ in segs:
                bounds.append(bounds[-1] + dur)
            continuous_target._params = (segs, bounds)
        segs, bounds = continuous_target._params

        for k, (dur, v) in enumerate(segs):
            if t < bounds[k + 1]:
                return v
        return 0.0

    T_total = 2.5
    steps = int(T_total / dt)

    continuous_target(0.0)
    _segs, _bounds = continuous_target._params

    print(f"随机目标 (覆盖全场景, 每次不同):")
    for k, (dur, v) in enumerate(_segs):
        print(f"  段{k}: t={_bounds[k]:.2f}~{_bounds[k+1]:.2f}s  目标={v:+.0f}  (时长{dur:.2f}s)")
    print()

    target_schedule = [(i * dt, continuous_target(i * dt)) for i in range(steps)]

    log_t = []
    log_target = []
    log_v_ref = []
    log_a_ref = []
    log_cmd = []
    log_actual = []

    v_act = 0.0
    tau_motor = 0.003

    for i in range(steps):
        t = i * dt
        target = continuous_target(t)

        v_ref, cmd = mc.update(target, v_act)
        a_ref = mc._get_a_ref()

        v_act += (cmd - v_act) / tau_motor * dt

        log_t.append(t)
        log_target.append(target)
        log_v_ref.append(v_ref)
        log_a_ref.append(a_ref)
        log_cmd.append(cmd)
        log_actual.append(v_act)

    import numpy as np
    a_ref_arr = np.array(log_a_ref)
    v_ref_arr = np.array(log_v_ref)
    cmd_arr = np.array(log_cmd)
    v_act_arr = np.array(log_actual)

    print("\n=== 边界检查统计 ===")
    print(f"  最大 |v_ref|      : {np.max(np.abs(v_ref_arr)):8.2f}  (目标范围 ±300)")
    print(f"  最大 |a_ref|      : {np.max(np.abs(a_ref_arr)):8.2f}  (amax=1000)")
    print(f"  最大 |cmd|        : {np.max(np.abs(cmd_arr)):8.2f}  (out_max=1.0)")
    print(f"  最大 |v_act|      : {np.max(np.abs(v_act_arr)):8.2f}")
    print(f"  最大 |v_ref - v_act|: {np.max(np.abs(v_ref_arr - v_act_arr)):8.2f}")

    dv_ref = np.abs(np.diff(v_ref_arr))
    max_dv_step = np.max(dv_ref)
    print(f"  单步最大 Δv_ref   : {max_dv_step:8.4f}  (dt=0.001s)")

    da_ref = np.abs(np.diff(a_ref_arr))
    max_da_step = np.max(da_ref)
    print(f"  单步最大 Δa_ref   : {max_da_step:8.4f}  (旧算法目标切换时会 >4000)")

    sign_changes = 0
    for i in range(1, len(v_ref_arr)):
        if v_ref_arr[i] * v_ref_arr[i-1] < 0:
            sign_changes += 1
    print(f"  v_ref 过零次数    : {sign_changes}")

    v_limit_check = max(abs(v) for _, v in _segs) * 1.1
    bad_v = np.sum(np.abs(v_ref_arr) > v_limit_check)
    bad_a = np.sum(np.abs(a_ref_arr) > 1000.0 * 1.5)
    bad_cmd = np.sum(np.abs(cmd_arr) > 1.0)
    print(f"  超范围 v_ref      : {bad_v} 帧 (目标峰值*1.1={v_limit_check:.0f})")
    print(f"  超范围 a_ref      : {bad_a} 帧 (amax*1.5=1500)")
    print(f"  超范围 cmd        : {bad_cmd} 帧 (out_max=1.0)")
    print(f"  NaN 帧            : {np.sum(~np.isfinite(v_ref_arr))}")

    print(f"\n  加速度连续性: 最大单步跳变 {max_da_step:.2f}")
    print(f"               阈值(旧算法会>4000): {'PASS' if max_da_step < 2000 else 'FAIL'}")

    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex='col')
    axL, axR = axes[:, 0], axes[:, 1]

    _segs, _bounds = continuous_target._params
    switch_times = _bounds[1:-1]

    jerk_arr = np.zeros_like(a_ref_arr)
    jerk_arr[1:] = np.diff(a_ref_arr) / dt

    axL[0].step(log_t, log_target, '-', color='gray', lw=1.8, alpha=0.8,
                where='post', label='target (step)')
    axL[0].plot(log_t, log_v_ref, '-', color='#4CAF50', lw=2.4,
                label='v_ref (S-curve)')
    axL[0].plot(log_t, log_actual, '-', color='#2196F3', lw=1.6,
                label='v_actual (motor)')
    for t_sw in switch_times:
        axL[0].axvline(t_sw, color='red', lw=1.0, ls='--', alpha=0.6)
    axL[0].text(0.005, 0.97, f'max|dv/dt|={max_dv_step:.3f} (limit {mc.amax*dt:.1f})',
                transform=axL[0].transAxes, fontsize=10, color='#4CAF50',
                va='top', bbox=dict(boxstyle='round', fc='white', alpha=0.8))
    axL[0].set_ylabel('Speed (mm/s)')
    axL[0].set_title(f'① 速度控制仿真 — target step vs S-curve vs actual   '
                     f'(v_peak={np.max(v_ref_arr):.0f}, '
                     f'lag_max={np.max(np.abs(v_ref_arr-v_act_arr)):.2f})',
                     fontsize=11)
    axL[0].legend(fontsize=9, loc='lower right')
    axL[0].grid(True, alpha=0.3)

    axL[1].plot(log_t, log_a_ref, '-', color='#FF9800', lw=2.0,
                label='a_ref (S-curve)')
    axL[1].axhline(mc.amax, color='red', lw=1.2, ls='--', alpha=0.6)
    axL[1].axhline(-mc.amax, color='red', lw=1.2, ls='--', alpha=0.6)
    axL[1].text(0.005, mc.amax, f'+amax={mc.amax:.0f}', fontsize=9,
                color='red', va='bottom')
    axL[1].text(0.005, -mc.amax, f'-amax={-mc.amax:.0f}', fontsize=9,
                color='red', va='top')
    axL[1].text(0.995, 0.97, f'max|a|={np.max(np.abs(a_ref_arr)):.0f} '
                f'(limit {mc.amax})',
                transform=axL[1].transAxes, fontsize=10, color='#FF9800',
                ha='right', va='top', bbox=dict(boxstyle='round', fc='white', alpha=0.8))
    for t_sw in switch_times:
        axL[1].axvline(t_sw, color='red', lw=1.0, ls='--', alpha=0.6)
    axL[1].axhline(y=0, color='gray', lw=0.6, alpha=0.5)
    axL[1].set_ylabel('Acceleration (mm/s^2)')
    axL[1].set_title(f'② 加速度 — S-curve reference, 必须在 ±amax={mc.amax:.0f} 内',
                     fontsize=11)
    axL[1].legend(fontsize=9, loc='lower right')
    axL[1].grid(True, alpha=0.3)

    axL[2].plot(log_t, jerk_arr, '-', color='#9C27B0', lw=1.8,
                label='jerk = da/dt')
    axL[2].axhline(y=0, color='gray', lw=0.6, alpha=0.5)
    for t_sw in switch_times:
        axL[2].axvline(t_sw, color='red', lw=1.0, ls='--', alpha=0.6)
    axL[2].text(0.995, 0.97, f'max|jerk|={np.max(np.abs(jerk_arr)):.0f}',
                transform=axL[2].transAxes, fontsize=10, color='#9C27B0',
                ha='right', va='top', bbox=dict(boxstyle='round', fc='white', alpha=0.8))
    axL[2].set_xlabel('Time (s)')
    axL[2].set_ylabel('Jerk (mm/s^3)')
    axL[2].set_title('③ Jerk — 加速度不能跳变 (这是 a 缓慢爬向 ±amax 的原因)',
                     fontsize=11)
    axL[2].legend(fontsize=9, loc='lower right')
    axL[2].grid(True, alpha=0.3)

    plt.tight_layout()

    print("\n=== Acceleration Continuity Test: 0->150->100 ===")
    mc2 = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000)
    dt = 0.001
    a_before_switch = None
    a_after_switch = None
    switch_time = 0.15

    for i in range(300):
        t = i * dt
        target = 150.0 if t < switch_time else 100.0
        v_ref, cmd = mc2.update(target, 0.0)
        a_ref = mc2._get_a_ref()

        if abs(t - switch_time) < dt * 0.5:
            a_before_switch = a_ref
        if abs(t - (switch_time + dt)) < dt * 0.5:
            a_after_switch = a_ref

    print(f"  Before switch: a_ref = {a_before_switch:.2f} mm/s^2")
    print(f"  After switch:  a_ref = {a_after_switch:.2f} mm/s^2")
    print(f"  Old algorithm: a_ref = 0.00 (acceleration RESET to zero!)")
    print(f"  New algorithm: acceleration CONTINUOUS (no reset)")
    print()

    print("=== Basic Test: 0->200 step response ===")
    mc3 = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000)
    for i in range(100):
        v_ref, _ = mc3.update(200.0, 0.0)
    print(f"  Reached target: v_ref={v_ref:.2f}, _active={mc3._active}")
    print()

    print("=== Multi-step Test: 0->150->100->200->0 ===")
    mc4 = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000)
    transitions = [
        (0.0, 0), (0.05, -150), (0.15, 100), (0.25, -200), (0.40, 0)
    ]
    prev_v = None
    prev_a = None
    for i in range(500):
        t = i * dt
        target = 0
        for tj, vj in transitions:
            if t >= tj:
                target = vj
        v_ref, _ = mc4.update(target, 0.0)
        a_ref = mc4._get_a_ref()

        for tj, vj in transitions:
            if abs(t - tj) < dt * 0.5 and prev_v is not None:
                print(f"  t={tj:.2f}s: v {prev_v:.1f}->{v_ref:.1f}, a {prev_a:.1f}->{a_ref:.1f}")
        prev_v, prev_a = v_ref, a_ref
    print()

    print("=== Vmax clamp Test: target=300 with v_max=200 ===")
    mc_clamp = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000,
                               v_max=200.0)
    v_peak = 0.0
    for i in range(int(1.0 / dt)):
        v_ref, _ = mc_clamp.update(300.0, None)
        v_peak = max(v_peak, abs(v_ref))
    print(f"  target=300, v_max=200 -> v_ref 峰值 {v_peak:.1f}  (≤200 {'PASS' if v_peak <= 200.1 else 'FAIL'})")

    print("\n=== Position Control Test (时间表): 0 -> +10mm -> 0 ===")
    mc_pos = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000,
                             v_max=60.0)
    mp_pos = Moveposition(mc_pos)
    # 电机模型与上方速度测试一致: 一阶响应, cmd 直接作为速度指令
    tau_motor = 0.003
    res = 0.001  # 1um 编码器分辨率
    v_act = 0.0
    pos = 0.0

    pos_log_t, pos_log_p, pos_log_v = [], [], []
    for target_pos in (10.0, 0.0):
        for i in range(int(4.0 / dt)):
            v_ref, cmd = mp_pos(target_pos, pos, v_act)
            v_act += (cmd - v_act) / tau_motor * dt
            pos += v_act * dt
            pos_meas = round(pos / res) * res  # 编码器量化
            pos_log_t.append(len(pos_log_p) * dt)
            pos_log_p.append(pos_meas)
            pos_log_v.append(v_ref)
            if mp_pos._arrived:
                break
        print(f"  target {target_pos:+7.2f}mm -> 最终位置 {pos_meas:+7.3f}mm"
              f"  (误差 {target_pos-pos_meas:+.3f}mm)")

    import numpy as np
    v_arr = np.array(pos_log_v)
    print(f"  总耗时: {len(pos_log_p)*dt:.3f}s, 峰值 v_ref: {np.max(np.abs(v_arr)):.1f} mm/s")

    print("\n=== 巡航测试: 0 -> +200mm (t = D/Ve - T) ===")
    mc_cruise = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000,
                                v_max=60.0)
    mp_cruise = Moveposition(mc_cruise)
    Ve = 0.85 * mc_cruise.v_max
    T = 2.1875 * Ve / mc_cruise.amax
    t_cruise = 200.0 / Ve - T
    print(f"  理论: Ve={Ve:.1f}, T={T:.3f}s, t_cruise={t_cruise:.3f}s, 总时长={2*T+t_cruise:.3f}s")
    v_act = 0.0
    pos = 0.0
    for i in range(int(6.0 / dt)):
        v_ref, cmd = mp_cruise(200.0, pos, v_act)
        v_act += (cmd - v_act) / tau_motor * dt
        pos += v_act * dt
        if mp_cruise._arrived:
            break
    print(f"  实测: 最终位置 {pos:+.3f}mm (误差 {200-pos:+.3f}mm), 用时 {i*dt:.3f}s")

    print("\n=== 中途改目标: 0 -> 10mm, 0.1s 后改为 5mm ===")
    mc_retgt = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000,
                               v_max=60.0)
    mp_retgt = Moveposition(mc_retgt)
    v_act = 0.0
    pos = 0.0
    target = 10.0
    for i in range(int(4.0 / dt)):
        if i * dt >= 0.1:
            target = 5.0
        v_ref, cmd = mp_retgt(target, pos, v_act)
        v_act += (cmd - v_act) / tau_motor * dt
        pos += v_act * dt
        if mp_retgt._arrived:
            break
    print(f"  目标中途 10→5mm: 最终位置 {pos:+.3f}mm (误差 {5-pos:+.3f}mm), 用时 {i*dt:.3f}s")

    print("\n=== 工作循环仿真 (真实电机: 摩擦+噪声+量化+负载抓放) ===")
    random.seed()   # 随机种子: 无参 = 每次运行噪声与目标序列都不同; 固定复现改 random.seed(42)
    # 随机目标序列 (每次运行不同), 覆盖: 大位移巡航 / 小位移三角 / 反向 / 回零
    d1 = random.uniform(80, 150)           # 段0: 大位移 (巡航)
    d2 = random.uniform(-120, -40)         # 段1: 回撤 (大概率反向)
    d3 = random.uniform(5, 30)             # 段2: 小位移 (可能三角剖)
    d4 = random.uniform(-80, -30)          # 段3: 反向
    seq = [round(d1, 1), round(d1 + d2, 1), round(d1 + d2 + d3, 1),
           round(d1 + d2 + d3 + d4, 1), 0.0]
    retarget_pos = round(seq[0] * random.uniform(0.55, 0.8), 1)   # 段0 中途改目标
    print(f"  随机序列: {seq}  (段0 中途改 {retarget_pos})")
    # 负载: 0.55s 挂上 1.2 (模拟抓取工件), 1.60s 放下 (disturb_mag 归零)
    mc_real = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=1000,
                              v_max=60.0, out_max=5.0)
    mp_real = Moveposition(mc_real)
    # cmd 是 mm/s 量纲. SimulatedMotor 的增益=1/mu_v, 时间常数 τ=m/mu_v:
    # 取 m=0.005, mu_v=1.0 → 增益 1 (cmd=51 → v≈50.8), τ=5ms (跟得上 S 曲线)
    # 编码器 10um 量化 + 20um 噪声 (常见参数, 随机差异肉眼可见)
    motor_real = SimulatedMotor(m=0.005, mu_c=0.2, mu_v=1.0,
                                out_sat=500.0,
                                resolution=0.01, noise_std=0.02,
                                disturb_mag=0.0, disturb_t=0.0)
    target = seq[0]
    seq_idx = 0
    retargeted = False
    pos, vel = 0.0, 0.0
    sim_t, sim_pos, sim_vref, sim_vel, sim_err = [], [], [], [], []
    for i in range(int(8.0 / dt)):
        t = i * dt
        if 0.55 <= t < 1.60:
            motor_real.disturb_mag = 1.2      # 挂上负载 (抓取)
        elif t >= 1.60:
            motor_real.disturb_mag = 0.0      # 放下负载
        if not retargeted and t >= 0.15:
            retargeted = True
            target = retarget_pos             # 中途改目标
        v_ref, cmd = mp_real(target, pos, vel)
        pos_meas, vel_meas = motor_real.step(cmd, dt, t=t)
        pos, vel = pos_meas, vel_meas
        sim_t.append(t)
        sim_pos.append(pos)
        sim_vref.append(v_ref)
        sim_vel.append(vel)
        sim_err.append(target - pos)
        if mp_real._arrived:
            print(f"  段{seq_idx}: target {target:+7.1f}mm -> 最终 {pos:+7.3f}mm"
                  f"  (误差 {target-pos:+.3f}mm), 用时 {i*dt:.2f}s")
            seq_idx += 1
            if seq_idx >= len(seq):
                break
            target = seq[seq_idx]

    axR[0].plot(sim_t, sim_pos, lw=2.0, color='#2196F3', label='pos (encoder)')
    for tg in seq:
        axR[0].axhline(tg, color='red', lw=1.0, ls='--', alpha=0.5)
    axR[0].axhline(80.0, color='red', lw=1.0, ls=':', alpha=0.5)  # 0.15s 改后的目标
    axR[0].axvspan(0.55, 1.60, color='orange', alpha=0.15, label='load on 0.55~1.60s')
    axR[0].set_ylabel('Position (mm)')
    axR[0].set_title('④ 位置控制仿真 — 随机工作循环 (巡航/三角/反向/回零), 负载抓放',
                     fontsize=11)
    axR[0].legend()
    axR[0].grid(True, alpha=0.3)
    axR[1].plot(sim_t, sim_vref, lw=2.0, color='#4CAF50', label='v_ref (S-curve)')
    axR[1].plot(sim_t, sim_vel, lw=1.2, color='#2196F3', alpha=0.8, label='v_meas (encoder)')
    axR[1].set_ylabel('Speed (mm/s)')
    axR[1].legend()
    axR[1].grid(True, alpha=0.3)
    axR[2].plot(sim_t, sim_err, lw=1.6, color='#9C27B0', label='target - pos')
    axR[2].axhline(0, color='gray', lw=0.8, alpha=0.6)
    axR[2].set_xlabel('Time (s)')
    axR[2].set_ylabel('Position error (mm)')
    axR[2].legend()
    axR[2].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('motor_controller_sim.png', dpi=120)
    plt.close()
    print("All sims merged -> motor_controller_sim.png")

    print("\n=== Torque Control Test: 0->0.8 阶跃 (v_max=1.0 限幅, amax=50 爬升率) ===")
    mc_tq = MotorController(kp=0.5, ki=0.02, kd=0.01, dt=0.001, amax=50.0,
                            v_max=1.0)
    tc_tq = TorqueControl(mc_tq, kd_v=0.5)
    # 状态机复用: 平滑/限幅; PID 不参与
    tq_prev, max_dtq, tq_peak = 0.0, 0.0, 0.0
    tq_ref = 0.0
    for i in range(int(1.0 / dt)):
        tq_ref, cmd = tc_tq(0.8, tq_ref)
        max_dtq = max(max_dtq, abs(tq_ref - tq_prev))
        tq_peak = max(tq_peak, abs(tq_ref))
        tq_prev = tq_ref
    print(f"  目标 0.8 -> torque_ref 峰值 {tq_peak:.4f}"
          f", 单帧最大 Δτ {max_dtq:.5f} (硬限 amax·dt = {mc_tq.amax * dt:.4f})"
          f" -> {'PASS' if tq_peak <= 0.8 + 1e-6 and max_dtq <= mc_tq.amax * dt + 1e-9 else 'FAIL'}")
    tc_tq.kd_v = 0.0               # 关阻尼: 传反馈也不触发 PID → cmd == torque_ref
    tq_ref, cmd = tc_tq(0.8, 0.7)
    print(f"  PID 不参与: 有反馈仍 cmd == torque_ref ({cmd - tq_ref:+.5f})"
          f" -> {'PASS' if abs(cmd - tq_ref) < 1e-9 else 'FAIL'}")
    tc_tq.kd_v = 0.5               # 恢复阻尼, 继续限幅/阻尼验证
    for i in range(int(1.0 / dt)):
        tq_ref, cmd = tc_tq(1.5, tq_ref)
    print(f"  目标 1.5 -> torque_ref 钳制在 {tq_ref:.4f} (v_max=1.0)"
          f" -> {'PASS' if abs(tq_ref - 1.0) < 1e-6 else 'FAIL'}")
    tq_ref, cmd = tc_tq(tq_ref, 1.0)
    print(f"  阻尼: torque_ref={tq_ref:.2f}, v=1.0 -> cmd={cmd:.4f} (期望 0.5000)"
          f" -> {'PASS' if abs(cmd - 0.5) < 1e-6 else 'FAIL'}")

    print()
    print("All tests passed!")
