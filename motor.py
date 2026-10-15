#!/usr/bin/env python3
"""
交互式电机仿真 — 控制性能实时可视化
===================================
输入线程实时下指令, 主循环 1ms 仿真 + 滚动窗口绘图.

命令:
  v 100        速度模式: 目标速度 100 mm/s
  p 50         位置模式: 目标位置 50 mm (Moveposition 映射 + S曲线 + PID)
  t 400 [kd]   力矩模式: 目标力矩 400 ∈ ±500 (S曲线平滑 + 限幅 + 可选速度
                阻尼 kd; 注意力矩电机静摩擦 ±150, |τ| 突破后才动)
  w 2.0        显示窗口宽度 (s, 最新点固定在中央 — 左半历史, 右半留白)
  s 2.0        仿真倍速 (实时×, 默认 1.0; 0.5 = 慢放, 3 = 快进)
  m            打印当前状态
  r            复位 (位置/控制器/应用层清零, 重新开始)
  q / Ctrl+C   退出并输出全程性能总结

显示 (三面板, 示波器式窗口 — 最新点固定中央, 左半历史右半留白):
  ① 速度: v 模式目标 (灰阶) 与 v_ref S曲线参考 (绿) vs vel 编码器实测 (蓝)
           (t 模式只显示实测速度 vel, 力矩曲线不占速度面板)
  ② 位置: 实测位置 (蓝) vs 当前位置目标 (红虚线, 仅 p 模式)
  ③ 加速度/cmd: a_ref (橙, ±amax 线, 右轴) + cmd (蓝, ±out_max 饱和线, 左轴);
                t 模式追加: 目标力矩 (灰) 与 τ_ref (绿) 叠加到本面板 —
                速度① / 位置② / 力矩+cmd③ 四路信号同时可见
  速度纵轴锁定: 以当前速度为中心, 固定 ±v_max/4 — 曲线变化始终在面板中间

实时校验 (窗口数据, 违规即红字):
  max|Δv_ref| ≤ amax·dt   单帧速度变化硬约束 (t 模式 = 力矩爬升率)
  max|a_ref| ≤ amax       加速度硬约束 (t 模式 = 力矩变化率)
  |cmd| ≤ out_max         输出限幅 (统计饱和帧数)
  max|vel| ≤ v_max        电机最大速度 (任何模式绝对不突破; t 模式靠机器钳速)
  max|a_act| ≤ amax      电机实际加速度硬限 (t 模式: 机器过载截断)

到位事件自动打印: 目标 / 最终位置 / 误差 / 用时; 退出时输出全程总结.
注意: 位置模式必须直接用 mp() 的返回值, 不要再调 mc.update() —
      mp 内部已完整执行 update, 再调用会 PID 积分重复/曲线双重平滑,
      测出的性能是假的.
"""
import sys
import threading
import time
from collections import deque

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
matplotlib.rcParams['axes.unicode_minus'] = False
# Windows 下 stdout 为管道/重定向时按 GBK 编码, τ(希腊)/✅/− 等字符会抛
# UnicodeEncodeError 中断仿真 (交互式控制台走宽字符 API, 不受影响)。
# 兜底: 无法编码的字符替换为 '?', 保证重定向/子进程捕获也能跑完。
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(errors='replace')

from motor_controller import MotorController, SimulatedMotor, Moveposition, TorqueControl

# ---------------- 共享状态 (输入线程写, 主循环读) ----------------
mode = "v"            # 'v' 速度模式 / 'p' 位置模式 / 't' 力矩模式
target_v = 0.0
target_p = 0.0
target_t = 0.0
win_sec = 1.0         # 滚动窗口宽度 (s)
speed_factor = 1.0    # 仿真倍速 (实时×)
quit_flag = False
reset_flag = False
last_mode = "v"        # 模式切换检测: 切换帧重置窗口/轨迹指标 (不跨模式混算)

def input_listener():
    global mode, target_v, target_p, target_t, win_sec, speed_factor, quit_flag, reset_flag
    while True:
        try:
            user_input = input("命令 [v 速度] [p 位置] [t 力矩] [w 窗口] [s 倍速] [m 状态] [r 复位] [q 退出] 或直接输入数值> ")
        except (EOFError, OSError):
            return
        parts = user_input.split()
        if not parts:
            continue
        try:
            c = parts[0].lower()
            if c == 'q':
                quit_flag = True
                print("-> 退出中, 汇总性能数据后结束")
                return
            if c == 'm':
                if mode == 'v':
                    tgt_str = f"速度 {target_v:.1f} mm/s"
                elif mode == 't':
                    tgt_str = f"力矩 {target_t:.2f} (阻尼 kd_v={tc.kd_v})"
                else:
                    tgt_str = f"位置 {target_p:.2f} mm"
                print(f"-> 模式={mode}  目标={tgt_str}"
                      f"  窗口={win_sec:.1f}s  倍速={speed_factor:.2f}×")
                continue
            if c == 'r':
                reset_flag = True
                print("-> 复位请求已接收 (下一帧生效)")
                continue
            if c == 'w' and len(parts) > 1:
                win_sec = max(0.2, min(10.0, float(parts[1])))
                print(f"-> 窗口宽度 -> {win_sec:.1f}s")
                continue
            if c == 's' and len(parts) > 1:
                speed_factor = max(0.1, min(10.0, float(parts[1])))
                print(f"-> 仿真倍速 -> {speed_factor:.2f}× 实时")
                continue
            if c == 'v' and len(parts) > 1:
                mode = "v"; target_v = float(parts[1])
                print(f"-> 【速度模式】目标速度 {target_v} mm/s")
                continue
            if c == 'p' and len(parts) > 1:
                mode = "p"; target_p = float(parts[1])
                print(f"-> 【位置模式】目标位置 {target_p} mm")
                continue
            if c == 't' and len(parts) > 1:
                mode = "t"; target_t = float(parts[1])
                if len(parts) > 2:
                    tc.kd_v = float(parts[2])
                    print(f"-> 【力矩模式】目标力矩 {target_t}  阻尼 kd_v={tc.kd_v}")
                else:
                    print(f"-> 【力矩模式】目标力矩 {target_t}")
                continue
            val = float(parts[0])
            if mode == "v":
                target_v = val
                print(f"-> 速度目标 -> {target_v} mm/s")
            elif mode == "t":
                target_t = val
                print(f"-> 力矩目标 -> {target_t}")
            else:
                target_p = val
                print(f"-> 位置目标 -> {target_p} mm")
        except ValueError:
            print("输入无效. 例: v 100 / p 30 / t 400 / w 2 / m / r / q")

threading.Thread(target=input_listener, daemon=True).start()

# ---------------- 仿真对象 ----------------
dt = 0.001
v_max = 800.0
amax = 1000.0
out_max = 500.0
mc = MotorController(kp=5.0, ki=0.1, kd=0.01, dt=dt, amax=amax, v_max=v_max,
                     out_max=out_max)
# 速度/位置模式的机器 = 已闭合内环的速度伺服抽象: 实际速度跟随 v_ref,
# 实际加速度 ≈ 参考加速度 ≤ amax (S曲线保证) — 物理极限由"参考+闭环"保障;
# 机器侧仍挂 v_max 作为最后一道硬件限速保险 (极端瞬态/扰动下防越线)。
# 注意: 不挂 a_max — 伺服抽象的内环才是加速度来源, 硬限会破坏跟踪。
motor = SimulatedMotor(m=0.005, mu_c=0.2, mu_v=1.0, out_sat=v_max,
                       resolution=0.01, noise_std=0.02, v_max=v_max)
mp = Moveposition(mc)
# 力矩模式: 复用同一个 mc 的 S 曲线状态机 (同构, PID 不参与);
# 电机侧按"力矩环驱动"建模 (电流环直驱 → 恒力矩 = 恒净力 → 匀加速),
# 区别于伺服驱动器 (速度环, μ_v=1.0) — 同一台电机, 驱动器模式不同.
# 摩擦必须与矩力量程同标度 (μ_c=100 ≈ 量程 20%): 才能看到
#   τ=0 且 v≠0 → 摩擦主动减速 (a≈−350 → 2s 级清零); τ<μ_c → 死区; τ>μ_c → 匀加速。
# μ_v=0.02 小粘性: 恒力矩≈匀加速, 且 τ=0 时对中高速段有额外减速。
# 这台力矩环机器没有内部控制环 — 电机的物理极限必须挂在机器模型上硬执行:
#   v_max = 电机最大速度: 到达即钳速 (否则恒力矩把速度冲到 2 万 ≫ 800);
#   amax  = 电机最大加速度: 请求的加速度超出即截断 (否则 τ=500 时 a≈1333>1000)。
# 力矩指令给多大, 机器也只是按自己的极限加速/减速 — 绝不可能突破 v_max/amax。
# mu_s=150 > mu_c=100: 真实静摩擦 — τ≤150 原地顶住不动, 破 150 后按动摩擦 100 滑动
motor_t = SimulatedMotor(m=0.3, mu_c=100.0, mu_s=150.0, mu_v=0.02,
                         out_sat=out_max, resolution=0.01, noise_std=0.02,
                         v_max=v_max, a_max=amax)
# 力矩钳幅 = 机器额定力矩 (out_sat=500), 不再借用 v_max 的"速度"语义
tc = TorqueControl(mc, kd_v=0.0, torque_max=motor_t.out_sat)
pos_tol = mp.pos_tol

# ---------------- 数据缓存 (滚动窗口 + 全程累计) ----------------
BUF = int(10.0 / dt)                       # 窗口上限 10s
times = deque(maxlen=BUF)
vrefs = deque(maxlen=BUF)
vacts = deque(maxlen=BUF)
aacts = deque(maxlen=BUF)      # 机器实际加速度 (t 模式物理极限校验)
cmds = deque(maxlen=BUF)
arefs = deque(maxlen=BUF)
ds = deque(maxlen=BUF)
poss = deque(maxlen=BUF)
run = {   # 全程累计: 退出总结用
    'frames': 0, 'max_dv': 0.0, 'max_a': 0.0, 'max_cmd': 0.0,
    'max_lag': 0.0, 'max_v': 0.0, 'max_a_act': 0.0, 'sat_frames': 0,
    'arrive_log': [],
}

# ---------------- 图形 ----------------
plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 11), sharex=True)
line_tgt, = ax1.plot([], [], '-', color='gray', lw=1.8, alpha=0.8,
                     label='目标 (v 模式)')
line_ref, = ax1.plot([], [], '-', color='#4CAF50', lw=2.2, label='v_ref (S曲线)')
line_act, = ax1.plot([], [], '--', color='#2196F3', lw=1.6, label='vel (编码器)')
ax1.set_ylabel('Speed (mm/s)')
ax1.legend(loc='lower right', fontsize=8)
ax1.grid(True, alpha=0.3)
# 两个信息框都放在右侧"未来"空白区 (右半边永远无曲线), 小字号不挡图
info_text = ax1.text(0.98, 0.985, "", transform=ax1.transAxes, va='top',
                     ha='right', fontsize=8,
                     bbox=dict(boxstyle='round', fc='white', alpha=0.8))
verdict_text = ax1.text(0.98, 0.38, "", transform=ax1.transAxes, va='center',
                        ha='right', fontsize=8,
                        bbox=dict(boxstyle='round', fc='white', alpha=0.8))

line_pos, = ax2.plot([], [], '-', color='#2196F3', lw=1.6, label='pos (mm)')
line_tgtp, = ax2.plot([], [], '--', color='red', lw=1.2, alpha=0.7,
                      label='目标位置 (p 模式)')
ax2.axhline(0, color='gray', lw=0.6, alpha=0.5)
ax2.set_ylabel('Position (mm)')
ax2.legend(loc='lower right', fontsize=8)
ax2.grid(True, alpha=0.3)

line_cmd, = ax3.plot([], [], '-', color='#2196F3', lw=1.2, alpha=0.75,
                     label='cmd (左轴)')
line_tq_tgt, = ax3.plot([], [], '-', color='gray', lw=1.4, alpha=0.7,
                        label='目标力矩 (t 模式)')
line_tq_ref, = ax3.plot([], [], '-', color='#4CAF50', lw=1.5,
                        label='τ_ref (S曲线, t 模式)')
ax3.axhline(+out_max, color='red', ls='--', lw=0.8, alpha=0.5)
ax3.axhline(-out_max, color='red', ls='--', lw=0.8, alpha=0.5)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('cmd (mm/s)')
ax3.legend(loc='lower right', fontsize=8)
ax3.grid(True, alpha=0.3)
ax3b = ax3.twinx()
line_aref, = ax3b.plot([], [], '-', color='#FF9800', lw=1.6,
                       label='a_ref (右轴)')
ax3b.axhline(+amax, color='orange', ls=':', lw=0.8, alpha=0.5)
ax3b.axhline(-amax, color='orange', ls=':', lw=0.8, alpha=0.5)
ax3b.set_ylabel('a_ref (mm/s^2)')
ax3b.legend(loc='upper right', fontsize=8)

PHASE_TXT = {'CRUISE': '巡航', 'BRAKE': '刹停', 'DONE': '停稳/静止'}

# ---------------- 主循环 ----------------
pos, vel = 0.0, 0.0
t = 0.0
arr_printed = False          # 当前目标是否已打印过"到位" (每个目标只报一次)
n_frames = 0                 # 本显示周期推进的帧数 (用于状态显示)
last_target_p = None
wall_anchor = time.perf_counter()   # 实时跟踪: 每个显示周期追回多少仿真帧

print("--- 实时仿真已启动, 性能校验与到位事件会自动打印 ---")
try:
    while not quit_flag:
        if mode != last_mode:
            # 参考信号语义变了 (速度/位置/力矩): 曲线窗口与轨迹指标重置,
            # Δv_ref 与 Δτ_ref 不能跨模式相减 — 否则模式切换帧出现假 FAIL
            old_mode = last_mode
            last_mode = mode
            # 同一台物理电机, 两种驱动器建模 (v/p=速度环伺服 motor,
            # t=力矩环 motor_t, 各持一份状态): 切换模式必须把旧实例的
            # 位置/速度续接到新实例 — 否则新实例从 0 出发, 位置/速度
            # 读数猛然归零 (假"丢位置" bug, 2026-09-02 修)
            if mode == 't':
                motor_t.set_state(pos, vel)
            elif old_mode == 't':
                motor.set_state(pos, vel)
            times.clear(); vrefs.clear(); vacts.clear(); aacts.clear()
            cmds.clear(); arefs.clear(); ds.clear(); poss.clear()
            run['max_dv'] = 0.0
            run['max_lag'] = 0.0
            arr_printed = False
            if mode == 'p':
                # 同时给计时起点赋值: 否则原到位块 (target_p != last_target_p)
                # 被跳过 → 首次到位打印 t_p_set 未定义而崩溃
                last_target_p = target_p
                t_p_set = t
        # ----- 按倍速追回仿真帧 (实时 1.0× = 每显示周期推进 ≈ 流逝的壁钟时间) -----
        n_steps = int(min(500.0 * speed_factor,
                          max(1.0, (time.perf_counter() - wall_anchor)
                              * speed_factor / dt)))
        for _ in range(n_steps):
            if mode == 'v':
                v_ref, cmd = mc.update(target_v, vel)
            elif mode == 't':
                # 力矩模式: 复用 mc.update 的 S曲线状态机 (v_ref = τ_ref);
                # 反馈只进 kd_v 阻尼 (PID 不参与: 力矩/速度无量纲可比)
                v_ref, cmd = tc(target_t, vel)
            else:
                # mp 内部已执行 mc.update: 直接用返回值, 禁止再调 update!
                v_ref, cmd = mp(target_p, pos, vel)
                if mp._arrived and not arr_printed:
                    err = target_p - pos
                    print(f"  ✅ 到位: 目标 {target_p:.3f}mm → 实际 {pos:.3f}mm"
                          f"  (误差 {err:+.3f}mm, 用时 {t - t_p_set:.2f}s)")
                    run['arrive_log'].append((target_p, pos, err, t - t_p_set))
                    arr_printed = True
            a_ref = mc._get_a_ref()
            motor_cur = motor_t if mode == 't' else motor
            pos_meas, vel_meas = motor_cur.step(cmd, dt, t=t)
            pos, vel = pos_meas, vel_meas
            a_act = motor_cur._a_true      # 机器实际加速度 (硬限裁剪后)
            if mode == 't':                # v/p 模式的伺服抽象无此物理极限语义
                run['max_a_act'] = max(run['max_a_act'], abs(a_act))

            run['frames'] += 1
            run['max_dv'] = max(run['max_dv'],
                                abs(v_ref - (vrefs[-1] if vrefs else v_ref)))
            run['max_a'] = max(run['max_a'], abs(a_ref))
            run['max_cmd'] = max(run['max_cmd'], abs(cmd))
            run['max_v'] = max(run['max_v'], abs(vel))   # 电机限速 Vmax 校验
            if mode != 't':   # t 模式无"跟踪误差"语义 (力矩 vs 速度不可比)
                run['max_lag'] = max(run['max_lag'], abs(v_ref - vel))
            if abs(cmd) >= 0.999 * out_max:
                run['sat_frames'] += 1

            times.append(t); vrefs.append(v_ref); vacts.append(vel)
            aacts.append(a_act)
            cmds.append(cmd); arefs.append(a_ref)
            ds.append(target_p - pos if mode == 'p' else 0.0)
            poss.append(pos)
            t += dt

        # ----- 目标变更跟踪 (到位计时起点 / 允许再次打印到位) -----
        if mode == 'p' and target_p != last_target_p:
            last_target_p = target_p
            t_p_set = t
            arr_printed = False

        if reset_flag:
            reset_flag = False
            motor.reset(0.0); motor_t.reset(0.0)
            mc.reset(); mp.reset(); tc.reset()
            pos = vel = 0.0; t = 0.0
            times.clear(); vrefs.clear(); vacts.clear(); aacts.clear()
            cmds.clear(); arefs.clear(); ds.clear(); poss.clear()
            run['frames'] = 0; run['arrive_log'].clear()
            arr_printed = False; last_target_p = None
            print("-> 已复位, 从 0 开始")
            continue

        # ----- 窗口指标 (numpy 切片, 快; 统计范围 = 可视历史 = win_sec/2) -----
        n = len(times)
        w = min(n, max(2, int(win_sec / 2.0 / dt)))
        tv = np.array(times); vr = np.array(vrefs); va = np.array(vacts)
        cd = np.array(cmds); ar = np.array(arefs); dd = np.array(ds)
        ps = np.array(poss)
        sl = slice(n - w, n)
        tvw = tv[sl]; vrw = vr[sl]; vaw = va[sl]; cdw = cd[sl]
        arw = ar[sl]; ddw = dd[sl]; ppw = ps[sl]
        dv = np.max(np.abs(np.diff(vrw))) if len(vrw) > 1 else 0.0
        a_max = np.max(np.abs(arw)) if len(arw) else 0.0
        sat = int(np.sum(np.abs(cdw) >= 0.999 * out_max))
        lag_i = np.max(np.abs(vrw - vaw)) if len(vrw) else 0.0
        dmax = float(np.max(np.abs(ddw))) if len(ddw) else 0.0
        aa = np.array(aacts)
        a_act_w = float(np.max(np.abs(aa[sl]))) if len(aa[sl]) else 0.0
        v_max_w = float(np.max(np.abs(vaw))) if len(vaw) else 0.0
        dv_ok = dv <= amax * dt * 1.0001
        a_ok = a_max <= amax * 1.0001
        v_ok = v_max_w <= v_max * 1.01   # 1% 容差: 编码器噪声/量化余量
        a_act_ok = a_act_w <= amax * 1.01   # 机器实际加速度硬限 (t 模式)

        # ----- 更新曲线 (示波器式: 最新点固定在窗口中央) -----
        half = win_sec / 2.0
        ax1.set_xlim(t - half, t + half)
        keep = tvw > t - half
        if mode == 'v':
            line_tgt.set_data(tvw[keep], np.full(int(keep.sum()), target_v))
        else:
            line_tgt.set_data([], [])
        if mode == 't':
            # 力矩曲线不占速度面板: ① 只留实测速度, 力矩叠加到 cmd 面板③
            line_ref.set_data([], [])
        else:
            line_ref.set_data(tvw, vrw)
        line_act.set_data(tvw, vaw)
        ax1.set_ylabel('Speed (mm/s)')
        # 速度纵轴锁定: 以当前速度为中心, 上下各 v_max/4
        ax1.set_ylim(vel - v_max / 4, vel + v_max / 4)

        # 图例按模式重建 (隐藏线不出现在图例里)
        if ax1.legend_ is not None:
            ax1.legend_.remove()
        ax1.legend(handles=([line_tgt] if mode in ('v', 't') else []) +
                   [line_ref, line_act], loc='lower right', fontsize=8)

        # 位置面板: 贴数据范围 (p 模式含目标线, 一眼看到当前在哪)
        line_pos.set_data(tvw, ppw)
        if mode == 'p':
            line_tgtp.set_data(tvw[keep], np.full(int(keep.sum()), target_p))
        else:
            line_tgtp.set_data([], [])
        p_all = [float(x) for x in ppw[keep]] if keep.any() and len(ppw[keep]) else []
        if mode == 'p':
            p_all.append(float(target_p))
        if p_all:
            lo, hi = min(p_all), max(p_all)
            pad = max(0.5, 0.10 * (hi - lo))
            ax2.set_ylim(lo - pad, hi + pad)
        if ax2.legend_ is not None:
            ax2.legend_.remove()
        ax2.legend(handles=[line_pos] + ([line_tgtp] if mode == 'p' else []),
                   loc='lower right', fontsize=8)

        line_cmd.set_data(tvw, cdw)
        line_aref.set_data(tvw, arw)
        if mode == 't':
            line_tq_tgt.set_data(tvw[keep], np.full(int(keep.sum()), target_t))
            line_tq_ref.set_data(tvw, vrw)          # v_ref 变量即 τ_ref
        else:
            line_tq_tgt.set_data([], [])
            line_tq_ref.set_data([], [])
        ax3.set_ylabel('cmd / 力矩' if mode == 't' else 'cmd (mm/s)')
        ax3b.set_ylabel('dτ_ref/dt' if mode == 't' else 'a_ref (mm/s^2)')
        if ax3.legend_ is not None:
            ax3.legend_.remove()
        ax3.legend(handles=[line_cmd] + ([line_tq_tgt, line_tq_ref]
                                         if mode == 't' else []),
                   loc='lower right', fontsize=8)
        ax3.set_ylim(-out_max * 1.12, out_max * 1.12)
        ax3b.set_ylim(-amax * 1.15, amax * 1.15)

        # ----- 状态文本 -----
        mode_txt = {'v': '速度', 'p': '位置', 't': '力矩'}[mode]
        sig = 'τ_ref' if mode == 't' else 'v_ref'
        tgt_txt = (f"{target_v:.1f} mm/s" if mode == 'v'
                   else f"{target_t:.2f}" if mode == 't'
                   else f"{target_p:.3f} mm")
        ff = mc.kff_vel * v_ref
        pid = cmd - ff
        if mode == 'v':
            extra = f"跟踪误差: {v_ref - vel:+.2f} mm/s (窗口 max {lag_i:.2f})"
        elif mode == 't':
            # 力矩环电机: 净力 = τ − μ_c − μ_v·v, μ_v 小 → 恒力矩 ≈ 匀加速
            v_ter = 0.0
            if abs(target_t) > motor_t.mu_s:
                # 理论终端速度, 再被硬件 Vmax 钳住 (驱动器限速保护)
                v_ter = min((abs(target_t) - motor_t.mu_c) / motor_t.mu_v,
                            v_max) * (1.0 if target_t >= 0 else -1.0)
            if abs(target_t) <= motor_t.mu_s:
                # |τ| ≤ 静摩擦: 破不了静摩擦, 原地顶住不动 — 旧公式显示
                # a≈(20−100)/0.3=负值假象, 改为直接说明死区
                a_txt = f"静摩擦死区 (±{motor_t.mu_s:.0f}): 原地不动"
                v_ter = 0.0
            else:
                a_txt = (f"a≈{(abs(target_t) - motor_t.mu_c) / motor_t.m:+.0f}"
                         f" (动摩擦 μ_c={motor_t.mu_c:.0f})")
            extra = (f"速度: {vel:.1f} mm/s  (力矩环: 理论终端 {v_ter:+.0f}, "
                     f"{a_txt})\n"
                     f"阻尼: kd_v={tc.kd_v}  力矩限幅 ±{tc.torque_max}"
                     f"  爬升率 {mc.amax:.0f}/s  硬件限速 ±{v_max:.0f}"
                     f"  硬件加速度 ±{amax:.0f}")
        else:
            extra = (f"剩余 d: {target_p - pos:+.3f} mm  (±{pos_tol} 死区)"
                     f"  阶段: {PHASE_TXT.get(mp._phase, mp._phase)}\n"
                     f"到位: {'是' if mp._arrived else '否'}"
                     f"  窗口 max|d|: {dmax:.3f} mm")
        now = time.perf_counter()
        cyc = max(1e-6, now - wall_anchor)
        wall_anchor = now
        if mode == 't':
            cmd_txt = f"cmd: {cmd:.2f} = τ_ref {v_ref:.2f} + 阻尼 {cmd - v_ref:+.2f}"
        else:
            cmd_txt = f"cmd: {cmd:.1f} = 前馈 {ff:.1f} + PID {pid:.1f}"
        info_text.set_text(
            f"模式: {mode_txt}   目标: {tgt_txt}\n"
            f"{sig}: {v_ref:.2f} | vel: {vel:.2f} mm/s\n"
            f"{extra}\n"
            f"{cmd_txt}"
            f"   (窗口 max|cmd| {np.max(np.abs(cdw)) if len(cdw) else 0:.1f}"
            f", 饱和帧 {sat})\n"
            f"显示窗: {win_sec:.1f}s (历史 {win_sec / 2:.1f}s)"
            f"  倍速: {speed_factor:.2f}× 实时   (实跑 {n_steps * dt / cyc:.2f}×)")
        ok_all = dv_ok and a_ok and sat == 0 and v_ok and \
            (a_act_ok if mode == 't' else True)
        verdict_text.set_text(
            f"Δ{sig} {dv:.3f}/{amax * dt:.2f}/帧 {'PASS' if dv_ok else 'FAIL'}\n"
            f"|d{sig}/dt| {a_max:.0f}/{amax:.0f}   {'PASS' if a_ok else 'FAIL'}\n"
            f"|v| {v_max_w:.1f}/{v_max:.0f}  {'PASS' if v_ok else 'FAIL'}\n"
            f"饱和帧 {sat}   {'PASS' if sat == 0 else 'FAIL'}"
            + (f"\n|a_act| {a_act_w:.0f}/{amax:.0f}  "
               f"{'PASS' if a_act_ok else 'FAIL'}" if mode == 't' else ""))
        verdict_text.set_color('#2E7D32' if ok_all else '#C62828')

        plt.pause(0.02)
except KeyboardInterrupt:
    pass

# ---------------- 退出总结 ----------------
plt.ioff()
fig.suptitle(f'Motor live sim — 结束快照 t={t:.2f}s, 总帧 {run["frames"]}',
             fontsize=12)
fig.savefig('motor_live.png', dpi=110)
plt.close(fig)
s = run
print("\n" + "=" * 62)
print(f" 全程性能总结 (仿真 {t:.2f}s, {s['frames']} 帧, {dt * 1000:.0f}ms/帧)")
print("-" * 62)
sig_sum = 'τ_ref' if mode == 't' else 'v_ref'
print(f"  S曲线参考 |{sig_sum}| 峰值    "
      f"{max((abs(v) for v in vrefs), default=0):8.2f}"
      f" {'力矩' if mode == 't' else 'mm/s'}")
print(f"  单帧最大 Δ{sig_sum}                  {s['max_dv']:8.3f}"
      f"  (硬限 amax·dt = {amax * dt:.2f}, "
      f"{'PASS' if s['max_dv'] <= amax * dt * 1.0001 else 'FAIL'})")
print(f"  变化率 |d{sig_sum}/dt| 峰值       {s['max_a']:8.1f}  "
      f"(硬限 {amax:.0f}, "
      f"{'PASS' if s['max_a'] <= amax * 1.0001 else 'FAIL'})")
print(f"  控制量 |cmd| 峰值             {s['max_cmd']:8.1f}"
      f"  (限幅 {out_max:.0f}, 饱和 {s['sat_frames']} 帧"
      f" = {100.0 * s['sat_frames'] / max(1, s['frames']):.2f}%)")
print(f"  |vel| 峰值 (编码器)           {s['max_v']:8.1f} mm/s"
      f"  (电机限速 Vmax {v_max:.0f}, "
      f"{'PASS' if s['max_v'] <= v_max * 1.01 else 'FAIL'})")
if mode == 't':
    print(f"  电机实际加速度 |a| 峰值       {s['max_a_act']:8.1f} mm/s^2"
          f"  (硬限 amax {amax:.0f}, "
          f"{'PASS' if s['max_a_act'] <= amax * 1.01 else 'FAIL'})")
if mode != 't':   # 力矩模式无"跟踪误差"语义
    print(f"  跟踪误差 |v_ref−vel| 峰值     {s['max_lag']:8.2f} mm/s")
if s['arrive_log']:
    print("-" * 62)
    errs = [e[2] for e in s['arrive_log']]
    print(f"  位置到位 {len(errs)} 次:")
    for tg, fp, err, t_arr in s['arrive_log']:
        print(f"    目标 {tg:+8.3f}mm → 实际 {fp:+8.3f}mm  "
              f"误差 {err:+7.3f}mm  用时 {t_arr:.2f}s")
    print(f"  平均 |误差| {np.mean(np.abs(errs)):.3f} mm")
print("=" * 62)
print("  末帧快照已保存 -> motor_live.png")
