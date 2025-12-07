import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ================= ⚙️ 参数配置区域 ⚙️ =================

# --- 1. 飞控连接设置 ---
CONNECTION_STRING = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- 2. 视觉阈值 (橙色目标) ---
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# --- 3. PID 控制参数 (关键修改) ---
MAX_SPEED = 0.25  # 🔴 绝对最大速度限制 (m/s)

# PID 参数 (需要根据实际飞行微调)
# P: 响应速度, I: 抵抗风力/消除静差, D: 抑制震荡
Kp = 0.0015
Ki = 0  # 🔴 I项：数值越小越安全，用于抗风
Kd = 0.0010  # 🔴 D项：用于刹车和防抖

# 积分限幅 (防止抗风过度导致刹不住车)
# 建议设为最大速度的 40%~60%
I_LIMIT = 0.15

# --- 4. 逻辑阈值 ---
ALIGN_THRESHOLD = 50
HOVER_DURATION = 4.0
MIN_AREA = 1000

# --- 5. 帧率与通信 ---
TARGET_FPS = 30
FRAME_INTERVAL = 1.0 / TARGET_FPS
CMD_FREQ_DIVIDER = 2

CAMERA_PATH = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"


# ========================================================

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
    # ---------------- Step 1: 连接飞控 ----------------
    print(f"🐢 [树莓派] 正在连接飞控...")
    try:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE, source_system=200, source_component=191)
        print("✅ 飞控连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    # ---------------- Step 2: 摄像头启动 ----------------
    print("📷 启动摄像头...")
    try:
        cap = cv2.VideoCapture(0)  # 也可以换回您的 CAMERA_PATH
        if not cap.isOpened():
            cap = cv2.VideoCapture(1)
    except:
        pass

    if not cap.isOpened():
        print("❌ 无法打开摄像头")
        vehicle.close()
        return

    cap.set(3, 640)
    cap.set(4, 480)
    CENTER_X, CENTER_Y = 320, 240
    print("✅ 摄像头就绪")

    # ---------------- Step 3: 等待 GUIDED 模式 ----------------
    print("⏳ 等待 GUIDED 模式...")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)
    print("🚀 GUIDED 模式激活！")

    # --- PID 状态变量初始化 ---
    hover_start_time = None
    kernel = np.ones((7, 7), np.uint8)

    # 时间与循环变量
    prev_time = time.time()
    loop_counter = 0

    # PID 误差累计变量
    integ_x, integ_y = 0.0, 0.0  # 积分累加值
    last_err_x, last_err_y = 0.0, 0.0  # 上一次误差

    try:
        while True:
            loop_start = time.time()
            loop_counter += 1

            # 1. 计算 dt (时间差) - PID 核心
            curr_time = time.time()
            dt = curr_time - prev_time
            if dt <= 0: dt = 0.001  # 防止除以0
            prev_time = curr_time

            # 2. 图像处理
            ret, frame = cap.read()
            if not ret: break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            velocity_x, velocity_y = 0, 0
            target_info = "搜索中..."
            err_x, err_y = 0, 0

            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > MIN_AREA:
                    ((tx, ty), radius) = cv2.minEnclosingCircle(c)

                    # 3. 计算误差
                    err_x = int(tx - CENTER_X)  # 对应横向移动 (Roll/Vy)
                    err_y = int(CENTER_Y - ty)  # 对应纵向移动 (Pitch/Vx)

                    # ==========================================
                    # 🔥 PID 核心计算逻辑 (含抗风处理)
                    # ==========================================

                    # --- X轴 PID (控制 Vy / 左右) ---
                    # P项
                    P_x = err_x * Kp
                    # I项 (积分) + 抗饱和 (Clamp)
                    integ_x += err_x * dt
                    integ_x = np.clip(integ_x, -I_LIMIT, I_LIMIT)  # 关键：限制I项最大值
                    I_x = integ_x * Ki
                    # D项 (微分)
                    D_x = ((err_x - last_err_x) / dt) * Kd

                    # --- Y轴 PID (控制 Vx / 前后) ---
                    # P项
                    P_y = err_y * Kp
                    # I项 (积分) + 抗饱和
                    integ_y += err_y * dt
                    integ_y = np.clip(integ_y, -I_LIMIT, I_LIMIT)  # 关键：限制I项最大值
                    I_y = integ_y * Ki
                    # D项 (微分)
                    D_y = ((err_y - last_err_y) / dt) * Kd

                    # --- 合成输出 ---
                    # 注意：图像的 X轴误差 控制无人机的 Vy (横向)
                    #       图像的 Y轴误差 控制无人机的 Vx (纵向)
                    raw_vel_y = P_x + I_x + D_x
                    raw_vel_x = P_y + I_y + D_y

                    # --- 总输出限幅 (安全第一) ---
                    velocity_y = np.clip(raw_vel_y, -MAX_SPEED, MAX_SPEED)
                    velocity_x = np.clip(raw_vel_x, -MAX_SPEED, MAX_SPEED)

                    # --- 更新历史误差 ---
                    last_err_x = err_x
                    last_err_y = err_y

                    # 4. 悬停逻辑
                    if abs(err_x) < ALIGN_THRESHOLD and abs(err_y) < ALIGN_THRESHOLD:
                        if hover_start_time is None: hover_start_time = time.time()
                        elapsed = time.time() - hover_start_time
                        target_info = f"锁定 | T-minus {HOVER_DURATION - elapsed:.1f}s"
                        if elapsed >= HOVER_DURATION:
                            print("\n✅ 悬停完成，降落！")
                            vehicle.mode = VehicleMode("LAND")
                            break
                    else:
                        hover_start_time = None
                        target_info = f"PID修正 | Vx:{velocity_x:.2f} Vy:{velocity_y:.2f}"
                else:
                    # 丢失目标时，清空积分，防止乱飘
                    integ_x, integ_y = 0.0, 0.0
            else:
                integ_x, integ_y = 0.0, 0.0

            # 5. 发送指令 (降频发送)
            if loop_counter % CMD_FREQ_DIVIDER == 0:
                if vehicle.mode.name == "GUIDED":
                    send_body_velocity(vehicle, velocity_x, velocity_y, 0)

            # 6. 显示信息
            print(f"\r{target_info} | Err X:{err_x} Y:{err_y} | I_term:{integ_x:.1f}" + " " * 5, end="")

            # FPS 限制
            elapsed = time.time() - loop_start
            if elapsed < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - elapsed)

    except KeyboardInterrupt:
        print("\n🛑 停止")
    finally:
        vehicle.close()
        cap.release()


if __name__ == "__main__":
    main()