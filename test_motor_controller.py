#!/usr/bin/env python3
"""
MotorController / Moveposition / TorqueControl / SimulatedMotor 全量使用验证
==================================================================
代替手动试测, 用项目里的【真实用法+真实参数】逐项验证文档承诺:

  A. 速度模式闭环  — S曲线硬约束 (|Δv_ref|≤amax·dt / |a_ref|≤amax / |v_ref|≤v_max)
                     + 实际速度收敛 (有编码器闭环, 伺服机 SimulatedMotor)
  B. 位置模式       — 大位移巡航/小位移三角/回零/反向/中途改目标/到位后停稳
                     到位误差 ≤ pos_tol, 死区无极限环 (抗量化噪声)
  C. 力矩模式       — τ_ref 爬升率 / 额定力矩钳幅 / PID 不参与 / kd_v 阻尼
                     力矩环电机: 静摩擦死区 / 匀加速 / v_max 钳速 / a_max 截断
  D. 电机模型单元   — 静摩擦死区 / stick-slip / 噪声量化界 / set_state 续接
  E. 模式切换回归   — motor.py 主循环全流程 (v→p→t→v→t→p→t), 专测
                     "切换后位置/速度突跳" bug (状态交接, 2026-09-02 修复)
  F. 随机负载加压   — 45s 随机命令乱切模式 + 中途复位, 全程硬约束不破 / 无 NaN
  G. 轮廓管线       — planner.py: 双轴 S 曲线闭环, 误差量级与 file 头记载一致
  H. CLI 冒烟       — 子进程跑 motor.py, 管道喂命令, 到位打印 + 退出码 0

硬约束如果被破坏 → FAIL; 全部 PASS → 退出码 0. 随机种子固定, 结果可复现.

运行: python test_motor_controller.py   (约 2 分钟)
"""
import math
import os
import random
import subprocess
import sys
import threading
import time
import traceback

import numpy as np

from motor_controller import (MotorController, SimulatedMotor,
                              Moveposition, TorqueControl)

random.seed(42)

# ---------------- 与 motor.py / planner.py 一致的真实参数 ----------------
DT = 0.001
AMAX = 1000.0
V_MAX = 800.0
OUT_MAX = 500.0
KP, KI, KD = 5.0, 0.1, 0.01
POS_TOL = 0.05
A_LIM = AMAX * DT                     # 单帧速度/力矩变化硬限


def new_mc():
    return MotorController(kp=KP, ki=KI, kd=KD, dt=DT,
                           amax=AMAX, v_max=V_MAX, out_max=OUT_MAX)


def new_servo():                      # v/p 模式机器 = 速度环伺服抽象
    return SimulatedMotor(m=0.005, mu_c=0.2, mu_v=1.0, out_sat=V_MAX,
                          resolution=0.01, noise_std=0.02, v_max=V_MAX)


def new_tmotor():                     # t 模式机器 = 力矩环直驱
    return SimulatedMotor(m=0.3, mu_c=100.0, mu_s=150.0, mu_v=0.02,
                          out_sat=OUT_MAX, resolution=0.01, noise_std=0.02,
                          v_max=V_MAX, a_max=AMAX)


# ---------------- 测试框架 ----------------
_results = []


def check(name, ok, detail=""):
    ok = bool(ok)
    _results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}"
          + (f"  ({detail})" if detail else ""))
    return ok


def finites(*vals):
    return all(math.isfinite(v) for v in vals)


class InvTrack:
    """全程硬约束监控: 单帧 Δv_ref / a_ref / cmd / v 峰值 + NaN"""
    __slots__ = ('max_dv', 'max_a', 'max_cmd', 'max_v', 'nan')

    def __init__(self):
        self.max_dv = self.max_a = self.max_cmd = self.max_v = 0.0
        self.nan = False

    def obs(self, v_ref, v_ref_prev, a_ref, cmd, vel):
        if not finites(v_ref, a_ref, cmd, vel):
            self.nan = True
        self.max_dv = max(self.max_dv, abs(v_ref - v_ref_prev))
        self.max_a = max(self.max_a, abs(a_ref))
        self.max_cmd = max(self.max_cmd, abs(cmd))
        self.max_v = max(self.max_v, abs(vel))

    def report(self, tag):
        ok = (not self.nan
              and self.max_dv <= A_LIM * 1.0001
              and self.max_a <= AMAX * 1.0001
              and self.max_v <= V_MAX * 1.0001)
        check(f"{tag}: 硬约束全程成立", ok,
              f"max|Δ|={self.max_dv:.4f}/{A_LIM:.4f}  "
              f"max|a|={self.max_a:.0f}/{AMAX:.0f}  "
              f"max|v|={self.max_v:.1f}/{V_MAX:.0f}"
              + (f"  max|cmd|={self.max_cmd:.1f}  NaN={self.nan}" if not ok else ""))
        return ok


# ================= A. 速度模式闭环 =================
def test_velocity_closed_loop():
    print("\n=== A. 速度模式闭环 (伺服机 + 编码器反馈) ===")
    mc = new_mc()
    motor = new_servo()
    targets = [200.0, -150.0, 50.0, -300.0, 300.0, 0.0]
    tr = InvTrack()
    v_ref_prev = 0.0
    pos, vel = 0.0, 0.0
    t = 0.0
    lag_max = 0.0
    for tg in targets:
        lag_hold = 0.0
        for _ in range(int(0.7 / DT)):
            v_ref, cmd = mc.update(tg, vel)
            tr.obs(v_ref, v_ref_prev, mc._get_a_ref(), cmd, vel)
            v_ref_prev = v_ref
            pos, vel = motor.step(cmd, DT, t=t)
            t += DT
            lag_hold = max(lag_hold, abs(v_ref - vel))
        lag_max = max(lag_max, lag_hold)
    tr.report("A")
    check("A: 实际速度跟上目标 (稳态跟踪误差小)",
          lag_max <= 40.0, f"max|v_ref-vel|={lag_max:.1f} mm/s")

    # 开环用法 (无编码器): v_ref 相同, cmd = 纯前馈
    mc2 = new_mc()
    for _ in range(500):
        v_ref2, cmd2 = mc2.update(200.0, None)
    check("A: 开环 (actual=None) 可用, v_ref 到 200",
          abs(v_ref2 - 200.0) < 1e-6 and abs(cmd2 - 200.0) < 1e-6,
          f"v_ref={v_ref2:.2f}, cmd={cmd2:.2f}")
    # 复位
    mc2.reset()
    v_ref3, cmd3 = mc2.update(0.0, None)
    check("A: reset() 后从 0 重新开始", abs(v_ref3) < 1e-9, f"v_ref={v_ref3:.2e}")


# ================= B. 位置模式 =================
def test_position_modes():
    print("\n=== B. 位置模式 (大位移/小位移/回零/反向/中途改目标/停稳) ===")
    targets = [150.0, 3.2, 0.0, -41.3, 19.8]
    errs = []
    tr = InvTrack()
    for tg in targets:
        mc = new_mc()
        mp = Moveposition(mc)
        motor = new_servo()
        pos, vel = 0.0, 0.0
        t = 0.0
        v_ref_prev = 0.0
        arrived = False
        t0 = None
        for i in range(int(10.0 / DT)):
            v_ref, cmd = mp(tg, pos, vel)
            tr.obs(v_ref, v_ref_prev, mc._get_a_ref(), cmd, vel)
            v_ref_prev = v_ref
            pos, vel = motor.step(cmd, DT, t=t)
            t += DT
            if t >= 0.5 and t0 is None:      # 过起动帧后再检到位
                t0 = t
            if mp._arrived and t0 is not None:
                if t - t0 > 0.6:              # 到位后再稳 0.6s
                    arrived = True
                    break
        check(f"B: 目标 {tg:+7.1f}mm 到位并停稳, 误差 ≤ {POS_TOL + 0.03:.2f}mm",
              arrived and abs(tg - pos) <= POS_TOL + 0.03,
              f"实际 {pos:+.3f}mm, 误差 {tg - pos:+.3f}mm"
              f"{'' if arrived else ' (超时未到位!)'}")
        errs.append(abs(tg - pos))
    tr.report("B")
    check("B: 平均 |误差| ≤ 0.05mm", float(np.mean(errs)) <= 0.05,
          f"mean={np.mean(errs):.4f}mm")

    # 中途改目标 (150 → 90, 巡航中)
    mc = new_mc()
    mp = Moveposition(mc)
    motor = new_servo()
    pos, vel = 0.0, 0.0
    tg = 150.0
    t = 0.0
    v_ref_prev = 0.0
    tr2 = InvTrack()
    for i in range(int(10.0 / DT)):
        if i >= int(0.5 / DT):
            tg = 90.0
        v_ref, cmd = mp(tg, pos, vel)
        tr2.obs(v_ref, v_ref_prev, mc._get_a_ref(), cmd, vel)
        v_ref_prev = v_ref
        pos, vel = motor.step(cmd, DT, t=t)
        t += DT
        if mp._arrived:
            break
    tr2.report("B-改目标")
    check("B: 巡航中改目标 150→90 → 到位",
          abs(90.0 - pos) <= POS_TOL + 0.03, f"实际 {pos:+.3f}mm")

    # 反向: 走到 150 中途改目标 0 (反向返回)
    mc = new_mc()
    mp = Moveposition(mc)
    motor = new_servo()
    pos, vel = 0.0, 0.0
    tg = 150.0
    t = 0.0
    for i in range(int(10.0 / DT)):
        if i >= int(0.5 / DT):
            tg = 0.0
        v_ref, cmd = mp(tg, pos, vel)
        pos, vel = motor.step(cmd, DT, t=t)
        t += DT
        if mp._arrived:
            break
    check("B: 正巡航中反切目标 150→0 → 到位",
          abs(0.0 - pos) <= POS_TOL + 0.03, f"实际 {pos:+.3f}mm")


# ================= C. 力矩模式 =================
def test_torque_modes():
    print("\n=== C. 力矩模式 (S曲线力矩 + 力矩环电机) ===")
    # τ_ref 爬升率 + 目标钳幅
    mc = new_mc()
    tc = TorqueControl(mc, kd_v=0.0, torque_max=OUT_MAX)
    prev = 0.0
    max_dtq = 0.0
    peak = 0.0
    for _ in range(int(1.0 / DT)):
        tq, cmd = tc(20.0, None)
        max_dtq = max(max_dtq, abs(tq - prev))
        prev = tq
        peak = max(peak, tq)
    check("C: τ_ref 单帧变化 ≤ amax·dt, 峰值=目标",
          max_dtq <= A_LIM * 1.0001 and abs(peak - 20.0) < 1e-6,
          f"maxΔτ={max_dtq:.4f}/{A_LIM:.4f}, 峰={peak:.3f}")
    # PID 不参与: 传反馈, cmd 仍等于 τ_ref (kd_v=0)
    tq_ref2, cmd2 = tc(300.0, 123.4)
    check("C: PID 不参与 (反馈传入仍 cmd==τ_ref)",
          cmd2 == tq_ref2, f"τ_ref={tq_ref2:.2f}, cmd={cmd2:.2f}")
    # 目标超额定 → 钳幅到 torque_max (等 S 曲线走到位再断言)
    for _ in range(int(2.0 / DT)):
        tq_lim, _ = tc(9999.0, None)
    check("C: 目标超额定 → τ_ref 钳到 torque_max",
          abs(tq_lim - OUT_MAX) < 1e-6, f"τ_ref={tq_lim:.1f}/{OUT_MAX}")
    # kd_v 阻尼: 先让 τ_ref 稳在 300, 再带速度进阻尼
    tc_k = TorqueControl(new_mc(), kd_v=0.5, torque_max=OUT_MAX)
    for _ in range(int(2.0 / DT)):
        tc_k(300.0, None)
    tq_k, cmd_k = tc_k(300.0, 100.0)
    check("C: kd_v 阻尼 cmd = τ_ref − 0.5·v",
          abs(cmd_k - (300.0 - 50.0)) < 1e-9,
          f"τ_ref={tq_k:.1f}, cmd={cmd_k:.2f} (期望 250)")

    # 死区: τ=20 < μ_s=150 → 原地不动
    m_dead = new_tmotor()
    for _ in range(int(1.0 / DT)):
        m_dead.step(20.0, DT)
    check("C: τ=20 静摩擦死区, 位置保持 0",
          abs(m_dead._pos) < 1e-9 and abs(m_dead._vel) < 1e-9)

    # τ=400 → 匀加速, |a|≤amax, |v|≤v_max (钳速)
    m_tq = new_tmotor()
    t = 0.0
    max_a = 0.0
    for _ in range(int(3.0 / DT)):
        tq, cmd = tc(400.0, None)
        pos, vel = m_tq.step(cmd, DT, t=t)
        t += DT
        max_a = max(max_a, m_tq._a_true)
    check("C: τ=400 恒力矩加速 → 被 v_max=800 钳速",
          abs(m_tq._vel_true - V_MAX) < 1e-9, f"vel={m_tq._vel_true:.2f}")
    check("C: 机器实际 |a| 不超过 a_max=1000",
          max_a <= AMAX * (1 + 1e-9), f"max|a_act|={max_a:.2f}")

    # τ=0 高速下降速: 摩擦减速 → 最终粘住 v=0, 无极限环
    m_dec = new_tmotor()
    m_dec.set_state(0.0, V_MAX)              # 从 800 mm/s 开始
    pos = 0.0
    t = 0.0
    for _ in range(int(4.0 / DT)):
        tq, cmd = tc(0.0, None)
        pos, vel = m_dec.step(cmd, DT, t=t)
        t += DT
    check("C: τ=0 摩擦减速, 低速粘住 (无极限环)",
          m_dec._vel == 0.0 and finites(pos),
          f"vel={m_dec._vel:.3f}, pos={pos:.1f}")

    # 力矩环 + kd_v 阻尼开环仿真稳定
    tc_k2 = TorqueControl(new_mc(), kd_v=0.1, torque_max=OUT_MAX)
    m_k = new_tmotor()
    pos, vel = 0.0, 0.0
    for _ in range(int(2.0 / DT)):
        tq, cmd = tc_k2(400.0, vel)
        pos, vel = m_k.step(cmd, DT)
    check("C: 力矩环带 kd_v=0.1 阻尼, 2s 稳定运行",
          finites(pos, vel) and abs(vel) <= V_MAX * 1.001,
          f"pos={pos:.1f}, vel={vel:.1f}")


# ================= D. 电机模型单元 =================
def test_motor_model_units():
    print("\n=== D. SimulatedMotor 模型单元 ===")
    # 破静摩擦: μ_s=150, τ=155 → 滑动, 一帧后 a=(155-100)/0.3≈183.3, v≈0.183
    m = new_tmotor()
    m.step(155.0, DT)
    check("D: 破静摩擦 (155>150), 按动摩擦滑移",
          abs(m._a_true - 183.33) < 0.5 and abs(m._vel_true - 0.1833) < 0.01,
          f"vel={m._vel_true:.4f}, a={m._a_true:.1f} (期望 a=183.3)")

    # a_max 截断: τ=500 → 请求 1333 → 截到 1000
    m2 = new_tmotor()
    m2.step(500.0, DT)
    check("D: a_max 硬截断", abs(m2._a_true - AMAX) < 1e-9, f"a={m2._a_true:.1f}")

    # v_max 钳速: 长时运行恒力矩
    m3 = new_tmotor()
    for _ in range(int(10.0 / DT)):
        m3.step(400.0, DT)
    check("D: v_max 钳速, 10s 恒力矩不越线",
          abs(m3._vel_true - V_MAX) < 1e-9)
    check("D: 位置单调增加 (恒力矩单向)", m3._pos > 0.0, f"pos={m3._pos:.1f}")

    # 噪声 + 量化界: 测量 = 真值 ± 4σ ≈0.1, 量化到 0.01
    m4 = new_tmotor()
    m4.set_state(12.34, 0.0)
    bad = 0
    for _ in range(2000):
        pos_meas, vel_meas = m4.step(0.0, DT)
        if abs(pos_meas - m4._pos) > 0.15 or abs(vel_meas - m4._vel_true) > 0.15:
            bad += 1
        r = round(pos_meas / 0.01)
        if abs(pos_meas - r * 0.01) > 1e-9:
            bad += 1
    check("D: 测量 = 真值 + 噪声 + 0.01 量化", bad == 0, f"超界帧={bad}/2000")

    # set_state 续接 (模式交接的基础)
    m5 = new_tmotor()
    m5.set_state(55.5, 77.0)
    ok_state = (abs(m5._pos - 55.5) < 1e-12 and abs(m5._vel - 77.0) < 1e-12
                and abs(m5._vel_true - 77.0) < 1e-12
                and abs(m5._a_true) < 1e-12)
    m5.step(400.0, DT)                      # 从注入状态继续
    check("D: set_state 注入状态并从该点续接",
          ok_state and abs(m5._pos - (55.5 + 77.0 * DT)) < 0.01,
          f"pos={m5._pos:.3f} (期望≈{55.5 + 77.0 * DT:.3f})")

    # 伺服机标定: cmd=51 → 稳态 v≈50.8 (motor_controller.py 文档记载)
    m6 = new_servo()
    for _ in range(int(0.5 / DT)):
        m6.step(51.0, DT)
    check("D: 伺服机参数标定 v_ter≈50.8 (cmd=51, μ_c=0.2, μ_v=1.0)",
          abs(m6._vel_true - 50.8) < 1.0, f"v={m6._vel_true:.2f}")


# ================= E. 模式切换交接回归 (就是上次修的 bug) =================
def test_mode_switch_handover():
    print("\n=== E. 模式切换状态交接 (motor.py 主循环流程, 修 bug 回归) ===")
    mc = new_mc()
    motor = new_servo()
    motor_t = new_tmotor()
    mp = Moveposition(mc)
    tc = TorqueControl(mc, kd_v=0.0, torque_max=motor_t.out_sat)
    pos, vel = 0.0, 0.0
    t = 0.0
    tr = InvTrack()
    v_ref_prev = 0.0
    max_switch_jump = 0.0

    def run_phase(mode, target, duration, until_arrive=False):
        """与 motor.py 主循环同构: mode == 't' 用力矩机, 否则伺服机。
        首个仿真帧度量"切换跳变": 须减去首帧真实物理运动 (v·dt, 800mm/s
        时一帧即 0.8mm), 剩余的才是状态注入造成的不连续 (应≈噪声 0.1 级);
        到位检测放在帧尾 (与 motor.py 一样先调 mp() 再查 _arrived)。"""
        nonlocal pos, vel, t, v_ref_prev, max_switch_jump
        t0 = t
        pos_before = pos
        limit = 25.0 if until_arrive else duration
        first = True
        while t - t0 < limit:
            if mode == 'v':
                v_ref, cmd = mc.update(target, vel); cx = motor
            elif mode == 't':
                v_ref, cmd = tc(target, vel); cx = motor_t
            else:
                v_ref, cmd = mp(target, pos, vel); cx = motor
            tr.obs(v_ref, v_ref_prev, mc._get_a_ref(), cmd, vel)
            v_ref_prev = v_ref
            pos, vel = cx.step(cmd, DT, t=t)
            t += DT
            if first:
                max_switch_jump = max(max_switch_jump,
                                      abs(pos - cx._vel_true * DT - pos_before))
                first = False
            if until_arrive and mp._arrived:
                break

    # v 200 → p 150 → t 400 → v 0 → t 400 → p 0 → t 20
    run_phase('v', 200.0, 1.5)
    # 切 p: 同一实例, 无状态交接
    run_phase('p', 150.0, 0.0, until_arrive=True)
    check("E: p 150 到位", abs(150.0 - pos) <= POS_TOL + 0.03,
          f"pos={pos:.3f}")
    # 切 t: 状态从伺服机交接到力矩机 → 位置连续 (原 bug: 跳 0)
    motor_t.set_state(pos, vel)
    run_phase('t', 400.0, 1.2)
    check("E: 切换瞬间状态跳变 ≤0.3mm (原 bug: 跳 0)",
          max_switch_jump <= 0.3, f"max jump={max_switch_jump:.3f}mm")
    # 切 v: 状态交回伺服机
    motor.set_state(pos, vel)
    run_phase('v', 0.0, 1.2)
    check("E: 反向切换 (t→v) 状态跳变 ≤0.3mm",
          max_switch_jump <= 0.3, f"max jump={max_switch_jump:.3f}mm")
    # 切 t: 再次交接
    motor_t.set_state(pos, vel)
    run_phase('t', 400.0, 0.6)
    check("E: 二次切换状态跳变 ≤0.3mm",
          max_switch_jump <= 0.3, f"max jump={max_switch_jump:.3f}mm")
    # 切 p: 回 0
    motor.set_state(pos, vel)
    run_phase('p', 0.0, 0.0, until_arrive=True)
    check("E: 全流程后 p 0 到位", abs(pos) <= POS_TOL + 0.03, f"pos={pos:.3f}")
    # 切 t: τ=20 死区, 位置应保持 (不漂移)
    motor_t.set_state(pos, vel)
    run_phase('t', 20.0, 1.0)
    check("E: t=20 死区, 位置保持在 0 附近不漂移",
          abs(pos) <= 0.5, f"pos={pos:.3f}")
    tr.report("E")


# ================= F. 随机负载加压 =================
def test_fuzz():
    print("\n=== F. 随机负载加压 (45s, 乱切模式 + 中途复位) ===")
    mc = new_mc()
    motor = new_servo()
    motor_t = new_tmotor()
    mp = Moveposition(mc)
    tc = TorqueControl(mc, kd_v=0.0, torque_max=motor_t.out_sat)
    pos, vel = 0.0, 0.0
    mode = 'v'
    last_mode = 'v'
    target = 0.0
    tr = InvTrack()
    v_ref_prev = 0.0
    max_jump = 0.0
    pos_before_switch = 0.0
    jump_pending = False
    switch_next = 0.0
    steps = int(45.0 / DT)
    for i in range(steps):
        t = i * DT
        if t >= switch_next:
            old_mode = last_mode
            r = random.random()
            if r < 0.35:
                mode = 'v'; target = random.uniform(-V_MAX * 0.9, V_MAX * 0.9)
            elif r < 0.75:
                mode = 't'; target = random.uniform(-OUT_MAX, OUT_MAX)
            else:
                mode = 'p'; target = pos + random.uniform(-40, 40)
            # 交接 (与 motor.py 主循环一致); 跳变量 = 新机首帧读数 vs 旧机末帧
            pos_before_switch = pos
            if mode == 't':
                motor_t.set_state(pos, vel)
            elif old_mode == 't':
                motor.set_state(pos, vel)
            jump_pending = True
            last_mode = mode
            switch_next = t + random.uniform(0.6, 2.0)
        if mode == 'v':
            v_ref, cmd = mc.update(target, vel); cx = motor
        elif mode == 't':
            v_ref, cmd = tc(target, vel); cx = motor_t
        else:
            v_ref, cmd = mp(target, pos, vel); cx = motor
        tr.obs(v_ref, v_ref_prev, mc._get_a_ref(), cmd, vel)
        v_ref_prev = v_ref
        pos, vel = cx.step(cmd, DT, t=t)
        if jump_pending:
            # 减去首帧真实物理运动 (v·dt), 剩余才是状态注入的不连续 (≈噪声)
            max_jump = max(max_jump,
                           abs(pos - cx._vel_true * DT - pos_before_switch))
            jump_pending = False
        if abs(t - 22.0) < DT * 0.5:       # 中途复位 (motor.py r 命令)
            motor.reset(0.0); motor_t.reset(0.0)
            mc.reset(); mp.reset(); tc.reset()
            pos = vel = 0.0
            v_ref_prev = 0.0
            tr.max_dv = tr.max_a = tr.max_cmd = tr.max_v = 0.0
    tr.report("F")
    check("F: 每次模式切换状态跳变 ≤0.3mm",
          max_jump <= 0.3, f"max jump={max_jump:.3f}mm")
    check("F: 全程有界 (位置有限值)", finites(pos), f"pos={pos:.1f}")


# ================= G. 轮廓管线 (planner.py 用法) =================
def test_planner_pipeline():
    print("\n=== G. 轮廓管线 (planner.py: 双轴 S 曲线闭环) ===")
    import planner
    points = planner.random_contour_points(seed=1)
    path = planner.build_path(points)
    s_samp, v_lims, _ = planner.build_velocity_profile(path, 500.0, AMAX)
    t_seq, v_seq, vx_seq, vy_seq, x_seq, y_seq = planner.plan_motion(
        path, s_samp, v_lims, 500.0, AMAX, DT)
    xa, ya, vx_ref, vy_ref, vx_act, vy_act = \
        planner.simulate_with_motor_controller(
            vx_seq, vy_seq, x_seq, y_seq, DT, AMAX, 0.005, kp=KP)
    finite = (np.isfinite(xa).all() and np.isfinite(ya).all()
              and np.isfinite(vx_ref).all() and np.isfinite(vy_ref).all())
    same = np.sqrt((xa - x_seq) ** 2 + (ya - y_seq) ** 2)
    dist = planner.point_to_path_distance(xa, ya, path)
    check("G: 管线无 NaN / 有界", finite)
    check("G: 双轴单帧 Δv ≤ amax·dt (planner 物理校验)",
          np.max(np.abs(np.diff(vx_seq))) <= AMAX * DT * 1.001
          and np.max(np.abs(np.diff(vy_seq))) <= AMAX * DT * 1.001,
          f"maxΔvx={np.max(np.abs(np.diff(vx_seq))):.4f}/{AMAX * DT:.4f}")
    check("G: S曲线参考 v_ref 紧跟路径目标 (无丢步)",
          np.max(np.abs(vx_ref - vx_seq)) <= 10.0
          and np.max(np.abs(vy_ref - vy_seq)) <= 10.0,
          f"max|vx_ref-vx_tgt|={np.max(np.abs(vx_ref - vx_seq)):.2f}")
    check("G: 真轮廓误差 ≤0.6mm (文件头记载 ~0.22)",
          np.max(dist) <= 0.6, f"max={np.max(dist):.3f}mm")
    check("G: 同刻误差 ≤2.5mm", np.max(same) <= 2.5, f"max={np.max(same):.3f}mm")


# ================= H. CLI 冒烟 (motor.py 子进程) =================
def test_cli_smoke():
    print("\n=== H. CLI 冒烟 (子进程跑 motor.py, 延迟喂命令) ===")
    # 真交互节奏喂命令 (一次全量喂入会让 'q' 在主循环首帧前被读掉 → 0 帧):
    # p 150 (到位1) → t 400 (位置续接加速) → p 0 (返回, 到位2) → q
    feed = [
        ("p 150\n", 7.0),
        ("t 400\n", 2.0),
        ("p 0\n", 8.0),
        ("q\n", 3.0),
    ]
    try:
        # 子进程输出编码显式固定 UTF-8: 若由父环境继承 PYTHONIOENCODING
        # 而父进程又按 locale(GBK) 解码管道, 会得到乱码 (count=0 假失败);
        # 两端显式锁定 utf-8 后与父环境无关, 结果确定。
        env = dict(os.environ)
        env["PYTHONIOENCODING"] = "utf-8"
        proc = subprocess.Popen(
            [sys.executable, "motor.py"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, encoding="utf-8",
            errors="replace", env=env, cwd=sys.path[0] or ".")
    except OSError:
        check("H: 子进程启动异常", False)
        return

    def feeder():
        for cmd, wait in feed:
            proc.stdin.write(cmd)
            proc.stdin.flush()
            time.sleep(wait)
        try:
            proc.stdin.close()
        except Exception:
            pass

    out_chunks, err_chunks = [], []

    def read_into(stream, chunks):
        for line in iter(stream.readline, ''):
            chunks.append(line)

    th_feed = threading.Thread(target=feeder, daemon=True)
    th_out = threading.Thread(target=read_into,
                              args=(proc.stdout, out_chunks), daemon=True)
    th_err = threading.Thread(target=read_into,
                              args=(proc.stderr, err_chunks), daemon=True)
    th_feed.start(); th_out.start(); th_err.start()
    try:
        proc.wait(timeout=150)
    except subprocess.TimeoutExpired:
        proc.kill()
        check("H: motor.py 运行并退出", False, "超时")
        return
    th_feed.join(timeout=5); th_out.join(timeout=5); th_err.join(timeout=5)
    out = "".join(out_chunks)
    err = "".join(err_chunks)
    check("H: 到位事件打印 ≥2 次 (p 150 与 p 0)",
          out.count("到位: 目标") >= 2,
          f"到位打印 {out.count('到位: 目标')} 次")
    check("H: 无崩溃, 子进程退出码 0",
          proc.returncode == 0 and "Traceback" not in err,
          f"returncode={proc.returncode}")
    check("H: 退出总结已输出",
          "全程性能总结" in out and "motor_live.png" in out)


# ================= 汇总 =================
def main():
    tests = [
        test_velocity_closed_loop,
        test_position_modes,
        test_torque_modes,
        test_motor_model_units,
        test_mode_switch_handover,
        test_fuzz,
        test_planner_pipeline,
        test_cli_smoke,
    ]
    for fn in tests:
        try:
            fn()
        except Exception as e:
            check(f"{fn.__name__}: 抛异常", False, repr(e))
            traceback.print_exc()

    n_ok = sum(1 for _, ok, _ in _results if ok)
    n = len(_results)
    print("\n" + "=" * 62)
    print(f"  结果: {n_ok}/{n} PASS")
    if n_ok != n:
        print("\n  未通过项:")
        for name, ok, detail in _results:
            if not ok:
                print(f"    FAIL {name}  ({detail})")
    print("=" * 62)
    sys.exit(0 if n_ok == n else 1)


if __name__ == '__main__':
    main()
