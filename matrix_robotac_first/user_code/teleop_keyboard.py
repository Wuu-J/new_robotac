#!/usr/bin/env python3
"""键盘遥控机器狗（手动驾驶，中期检查用）

用法（先启动仿真 ./run_sim.sh，再另开终端）:
    python3 teleop_keyboard.py
    python3 teleop_keyboard.py --dog-ip 192.168.234.1   # 真机

按键（按一次锁定速度，再按 Space 停止；方向键与 WASD 等效）:
    W / ↑  前进          S / ↓  后退
    A / ←  左转          D / →  右转
    E       左移(vy+)    R       右移(vy-)
    Space   急停 (move(0,0,0))
    L       趴下 (lieDown)
    Q / Esc 退出（先停稳再趴下）

设计要点（均来自注意事项.md 红线约束）:
    - 命令以 20Hz 持续发送（3 秒无 SDK 数据机器狗会自动趴下，循环间隔必须 <3s）
    - 速度避开死区: vx ±0.05~3, vy ±0.1~1, yaw ±0.02~3 m/s
    - 状态机约束: 切动作（standUp/lieDown）前先 move(0,0,0) 停稳
    - 仿真 initRobot 固定 127.0.0.1:43988
"""
import argparse
import os
import platform
import select
import sys
import termios
import time
import tty

# ---------------------------------------------------------------- SDK 导入
SDK_LIB = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'deps', 'zsibot_sdk', 'lib', 'zsl-1',
    platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')))
sys.path.insert(0, SDK_LIB)
import mc_sdk_zsl_1_py

# ---------------------------------------------------------------- 参数
V_FWD = 0.3      # 前进/后退速度 m/s（死区 ±0.05 ~ 3）
V_YAW = 0.5      # 转向角速度 rad/s（死区 ±0.02 ~ 3）
V_STRAFE = 0.3   # 横移速度 m/s（死区 ±0.1 ~ 1）
LOOP_HZ = 20.0   # 命令发送频率，建议 20~50Hz

# 横移方向约定：SDK 机器人坐标系「前X 左Y 上Z」，vy 正 = 左移。
# 若实机/仿真发现方向相反，交换下方 E/R 两个按键的符号即可。
KEY_ACTIONS = {
    'w': ('vx', +V_FWD), 's': ('vx', -V_FWD),
    'a': ('yaw', +V_YAW), 'd': ('yaw', -V_YAW),
    'e': ('vy', +V_STRAFE), 'r': ('vy', -V_STRAFE),
    '\x1b[A': ('vx', +V_FWD), '\x1b[B': ('vx', -V_FWD),   # ↑ ↓
    '\x1b[D': ('yaw', +V_YAW), '\x1b[C': ('yaw', -V_YAW),  # ← →
    ' ': ('stop', 0),
}

ERROR_CODES = {
    0x3013: '速度命令过大/超限',
    0x3012: '电机数据丢失',
    0x3011: '电机故障',
    0x3010: '电机失能',
    0x3009: '电机角度超限',
    0x3007: '状态机切换失败',
}

CTRL_MODES = {0: '阻尼', 1: '站立', 10: '趴下(短时后自由)', 18: '移动',
              21: '动作', 51: '趴下'}


def read_key(fd, timeout):
    """非阻塞读一个按键；方向键返回 '\x1b[A' 等形式。无输入返回 None。"""
    if not select.select([fd], [], [], timeout)[0]:
        return None
    c = os.read(fd, 1).decode(errors='ignore')
    if c == '\x1b':
        seq = c + os.read(fd, 2).decode(errors='ignore')
        return seq if seq in KEY_ACTIONS else '\x1b'
    return c.lower()


def main():
    ap = argparse.ArgumentParser(description='键盘遥控机器狗')
    ap.add_argument('--local-ip', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=43988)
    ap.add_argument('--dog-ip', default='127.0.0.1')  # 真机: 192.168.234.1
    args = ap.parse_args()

    if not sys.stdin.isatty():
        print('[ERROR] 需要在终端中运行（键盘输入）')
        return 1

    print(f'[INFO] 连接 SDK: local={args.local_ip}:{args.port} dog={args.dog_ip}')
    app = mc_sdk_zsl_1_py.HighLevel()
    app.initRobot(args.local_ip, args.port, args.dog_ip)
    time.sleep(1)
    try:
        if not app.checkConnect():
            print('[WARN] checkConnect() 返回 False，继续尝试（若控制无效请检查仿真是否启动）')
        else:
            print('[INFO] 连接成功')
    except AttributeError:
        print('[WARN] 无 checkConnect 接口，跳过连接检查')

    print('[INFO] 站立中...')
    app.standUp()
    time.sleep(3)
    state = 'standing'

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    vx = vy = yaw = 0.0
    last_status = 0.0
    print('[INFO] 就绪。按键: WASD/方向键=动 Space=停 L=趴下 Q=退出')
    try:
        while True:
            key = read_key(fd, 1.0 / LOOP_HZ)

            if key == 'q' or key == '\x1b':
                break
            if key == 'l':
                app.move(0, 0, 0); time.sleep(0.5)
                ret = app.lieDown()
                print(f'[INFO] 趴下指令返回 {ret:#x} ({ERROR_CODES.get(ret, "正常") if ret else "正常"})')
                state = 'down'; vx = vy = yaw = 0.0
            elif key in KEY_ACTIONS:
                axis, val = KEY_ACTIONS[key]
                if axis == 'stop':
                    vx = vy = yaw = 0.0
                else:
                    if state == 'down':          # 趴下时按移动键 → 先自动站立
                        print('[INFO] 自动站立...')
                        app.standUp(); time.sleep(3)
                        state = 'standing'
                    if axis == 'vx': vx = val
                    elif axis == 'vy': vy = val
                    else: yaw = val
                print(f'[INFO] 目标速度 vx={vx:+.2f} vy={vy:+.2f} yaw={yaw:+.2f}')

            ret = app.move(vx, vy, yaw)
            if ret != 0:
                print(f'[WARN] move 返回 {ret:#x}: {ERROR_CODES.get(ret, "未知错误")}')

            now = time.time()
            if now - last_status >= 5.0:
                last_status = now
                mode = app.getCurrentCtrlmode()
                print(f'[STATUS] vx={vx:+.2f} vy={vy:+.2f} yaw={yaw:+.2f} '
                      f'| ctrl_mode={mode} ({CTRL_MODES.get(mode, "未知")})')
    except KeyboardInterrupt:
        print('\n[INFO] 收到 Ctrl+C，安全退出')
    finally:
        # 状态机约束：先停稳，再趴下
        app.move(0, 0, 0)
        time.sleep(0.5)
        if state == 'standing':
            app.lieDown()
            time.sleep(0.3)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        print('[INFO] 已停止并趴下，退出')


if __name__ == '__main__':
    sys.exit(main())
