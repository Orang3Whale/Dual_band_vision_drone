import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ================= 🐢 树莓派专用版参数设置 🐢 =================

# 1. 连接 (树莓派通常是通过 USB 连接飞控，如果是 GPIO 请改为 '/dev/ttyAMA0')
CONNECTION_STRING = '/dev/ttyACM0'
BAUD_RATE = 57600

# 2. 视觉阈值 (橙色)
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# 3. 速度控制
MAX_SPEED = 0.25
Kp_X = 0.001
Kp_Y = 0.001

# 4. 逻辑阈值
ALIGN_THRESHOLD = 40
HOVER_DURATION = 4.0


# =========================================================

def send_body_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)


def main():
    # 1. 连接飞控
    print(f"🐢 [树莓派] 正在连接飞控: {CONNECTION_STRING} ...")
    try:
        # 树莓派上启动可能需要一点时间，wait_ready 很有必要
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE)
        print("✅ 飞控连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    # 2. 初始化摄像头
    cap = cv2.VideoCapture(0)
    # 降低分辨率以减轻树莓派 CPU 负担，提高帧率
    cap.set(3, 640)
    cap.set(4, 480)
    CENTER_X, CENTER_Y = 320, 240  # 中心点随分辨率改变

    print("摄像头已启动。正在等待 GUIDED 模式指令...")

    # --- 【关键修改】删除 input()，直接进入等待循环 ---
    # 树莓派上电启动脚本后，会卡在这个循环里，直到你用遥控器切模式

    while vehicle.mode.name != "GUIDED":
        print(f"\r⏳ [待机中] 当前模式: {vehicle.mode.name} (请用遥控器切到 GUIDED)...", end="")
        time.sleep(1)  # 待机时降低检查频率，省电省CPU

    print("\n✅ 检测到 GUIDED 模式，视觉控制逻辑已接管！")

    hover_start_time = None

    try:
        while True:
            # 检查模式安全锁：如果飞行中你切回 LOITER，脚本必须立即停止发送指令
            if vehicle.mode.name != "GUIDED":
                print("\n⚠️ 模式已变更，脚本暂停控制 (等待切回 GUIDED)...")
                while vehicle.mode.name != "GUIDED":
                    time.sleep(1)
                print("\n✅ 重新接管控制！")
                hover_start_time = None  # 重置计时

            ret, frame = cap.read()
            if not ret:
                # 防止摄像头掉线导致死循环报错，尝试重连或退出
                print("❌ 摄像头画面丢失")
                break

            # --- 视觉处理 (无绘图版) ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))

            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            velocity_x = 0
            velocity_y = 0

            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 1000:
                    ((tx, ty), radius) = cv2.minEnclosingCircle(c)

                    # 只要坐标计算，不需要 cv2.circle 绘图，节省 CPU
                    err_x = tx - CENTER_X
                    err_y = CENTER_Y - ty

                    # PID 计算
                    raw_vx = err_y * Kp_X
                    raw_vy = err_x * Kp_Y
                    velocity_x = np.clip(raw_vx, -MAX_SPEED, MAX_SPEED)
                    velocity_y = np.clip(raw_vy, -MAX_SPEED, MAX_SPEED)

                    # 悬停判定
                    is_aligned = abs(err_x) < ALIGN_THRESHOLD and abs(err_y) < ALIGN_THRESHOLD

                    if is_aligned:
                        if hover_start_time is None:
                            hover_start_time = time.time()

                        elapsed = time.time() - hover_start_time

                        # 仅保留终端打印用于日志记录
                        print(
                            f"\r[锁定] 倒计时 {HOVER_DURATION - elapsed:.1f}s | Vx:{velocity_x:.2f} Vy:{velocity_y:.2f}",
                            end="")

                        if elapsed >= HOVER_DURATION:
                            print("\n✅ 悬停达成！执行降落...")
                            send_body_velocity(vehicle, 0, 0, 0)
                            time.sleep(0.5)
                            vehicle.mode = VehicleMode("LAND")
                            break  # 任务结束，退出循环
                    else:
                        hover_start_time = None
                        print(f"\r[修正] 误差 X:{int(err_x)} Y:{int(err_y)} | Vx:{velocity_x:.2f}", end="")
            else:
                hover_start_time = None
                print(f"\r[搜索] 未发现目标...", end="")

            # 发送指令
            send_body_velocity(vehicle, velocity_x, velocity_y, 0)

            # --- 【关键修改】删除 cv2.imshow 和 cv2.waitKey ---
            # 树莓派后台运行时不需要显示，也不需要按键退出
            # 只是为了防止 CPU 占用过满，稍微 sleep 一个极小值给系统喘息
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 程序被强制终止")

    except Exception as e:
        print(f"\n❌ 运行时发生错误: {e}")

    finally:
        print("正在清理资源...")
        # 刹车
        send_body_velocity(vehicle, 0, 0, 0)
        cap.release()
        # cv2.destroyAllWindows() # 已删除，不需要了
        vehicle.close()
        print("程序已退出。")


if __name__ == "__main__":
    main()