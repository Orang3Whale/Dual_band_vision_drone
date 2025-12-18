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
H_MIN, S_MIN, V_MIN = 14 , 125, 150
H_MAX, S_MAX, V_MAX = 179, 255, 255

# --- 3. PD 飞行控制参数 (已添加 D 项) ---
MAX_SPEED = 1.2 # 最大飞行速度 (m/s)

# P (比例): 决定修正的快慢
Kp_X = 0.0045
Kp_Y = 0.0045

# D (微分): 决定刹车的力度 (防抖动、防过冲)
# 如果发现飞机靠近中心时抖动厉害，减小这个值
Kd_X = 0.0022
Kd_Y = 0.0022

ALPHA = 0.75  # D 项低通滤波系数 (0~1)，值越大响应越快，但抖动也越明显
# --- 4. 逻辑阈值 ---
ALIGN_THRESHOLD = 100  # 像素误差小于此值认为对准
HOVER_DURATION = 1.5  # 悬停保持时间 (秒)
MIN_AREA = 1000  # 最小识别面积

# --- 5. 帧率与通信限制 ---
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
    print(f"🐢 [树莓派] 正在通过 USB 连接飞控: {CONNECTION_STRING} ...")
    try:
        vehicle = connect(
            CONNECTION_STRING,
            wait_ready=True,
            baud=BAUD_RATE,
            source_system=200,
            source_component=191
        )
        print("✅ 飞控连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    # ---------------- Step 2: 智能启动摄像头 ----------------
    print("📷 正在启动摄像头...")
    try:
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
    except:
        pass

    if not cap.isOpened():
        print("⚠️ 尝试备用摄像头 Index 0 ...")
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ 致命错误：找不到任何可用的摄像头！")
        vehicle.close()
        return

    cap.set(3, 640)
    cap.set(4, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    CENTER_X, CENTER_Y = 320, 240
    print(f"✅ 摄像头就绪 (Backend: {cap.getBackendName()})")

    # ---------------- Step 3: 等待 GUIDED 模式 ----------------
    print("⏳ 等待切入 GUIDED 模式...")
    last_print_time = 0
    while vehicle.mode.name != "GUIDED":
        if time.time() - last_print_time > 2:
            print(f"当前模式: {vehicle.mode.name} (等待 GUIDED)...")
            last_print_time = time.time()
        time.sleep(0.5)

    print("🚀 GUIDED 模式激活！视觉系统接管控制！")

    # --- 变量初始化 ---
    hover_start_time = None
    kernel = np.ones((7, 7), np.uint8)
    
    # ⏱️ PD 控制所需的时间和误差变量
    prev_time = time.time()
    last_err_x = 0.0
    last_err_y = 0.0
    d_lpf_x = 0    # D 项低通滤波历史
    d_lpf_y = 0    # D 项低通滤波历史
     # D 项低通滤波系数
    loop_counter = 0

    try:
        while True:
            loop_start_time = time.time()
            loop_counter += 1

            # 1. 计算 dt (时间差) - 这里的 dt 专门用于 PID 计算
            curr_time = time.time()
            dt = curr_time - prev_time
            if dt <= 0: dt = 0.001 # 防止除以零
            prev_time = curr_time

            # --- 安全检查 ---
            if vehicle.mode.name != "GUIDED":
                print("⚠️ 模式变更，暂停控制...")
                send_body_velocity(vehicle, 0, 0, 0)
                while vehicle.mode.name != "GUIDED":
                    time.sleep(1)
                print("✅ 恢复控制！")
                # 恢复控制时重置 D 项历史，防止瞬间 D 值过大
                last_err_x, last_err_y,d_lpf_x,d_lpf_y = 0, 0, 0, 0
                hover_start_time = None
                prev_time = time.time() # 防止 dt 计算错误
                continue # 跳过本次循环，重新开始

            # --- 读取画面 ---
            ret, frame = cap.read()
            if not ret: break

            # ================= 核心 OpenCV 算法 =================
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
                    err_x = int(tx - CENTER_X)
                    err_y = int(CENTER_Y - ty)
                   
                    
          
                 
                    
                    # --- X轴 PD (控制 Vy / 左右) ---
                    # P项: 误差 * 比例
                    P_x = err_x * Kp_X
                    # D项: (当前误差 - 上次误差) / 时间差 * 微分系数
                    raw_diff_x = (err_x - last_err_x) / dt
                    d_lpf_x = (ALPHA * raw_diff_x) + ((1.0 - ALPHA) * d_lpf_x)
                    D_x = d_lpf_x * Kd_X
                    
                    # --- Y轴 PD (控制 Vx / 前后) ---

                    P_y = err_y * Kp_Y
                    raw_diff_y = (err_y - last_err_y) / dt
                    d_lpf_y = (ALPHA * raw_diff_y) + ((1.0 - ALPHA) * d_lpf_y)
                    D_y = d_lpf_y * Kd_Y

                    # 合成 PD 输出
                    # 注意：图像X轴误差 -> 控制无人机Y轴速度
                    #       图像Y轴误差 -> 控制无人机X轴速度
                    raw_vel_y = P_x + D_x
                    raw_vel_x = P_y + D_y

                    # 更新历史误差 (为下一轮 D 项计算做准备)
                    last_err_x = err_x
                    last_err_y = err_y

                    # 输出限幅
                    velocity_x = np.clip(raw_vel_x, -MAX_SPEED, MAX_SPEED)
                    velocity_y = np.clip(raw_vel_y, -MAX_SPEED, MAX_SPEED)

                    # 悬停判定
                    if abs(err_x) < ALIGN_THRESHOLD and abs(err_y) < ALIGN_THRESHOLD:
                        if hover_start_time is None:
                            hover_start_time = time.time()
                        elapsed = time.time() - hover_start_time
                        target_info = f"锁定 | 倒计时 {HOVER_DURATION - elapsed:.1f}s"

                        if elapsed >= HOVER_DURATION:
                            print("\n✅ 悬停完成，执行降落！")
                            send_body_velocity(vehicle, 0, 0, 0)
                            vehicle.mode = VehicleMode("LAND")
                            break
                    else:
                        hover_start_time = None
                        target_info = f"PD修正 | Vx:{velocity_x:.2f} Vy:{velocity_y:.2f}"
                else:
                    # 丢失目标时重置 D 项历史
                    last_err_x, last_err_y,d_lpf_x,d_lpf_y = 0, 0, 0, 0
            else:
                last_err_x, last_err_y,d_lpf_x,d_lpf_y = 0, 0, 0, 0

            # 通信降频
            if loop_counter % CMD_FREQ_DIVIDER == 0:
                send_body_velocity(vehicle, velocity_x, velocity_y, 0)

            # ================= FPS 限制与显示 =================
            process_duration = time.time() - loop_start_time
            if process_duration < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - process_duration)

            # 打印调试信息 (包含 FPS)
            fps = 1.0 / dt if dt > 0 else 0
            print(f"\rFPS: {fps:.1f} | {target_info} | Err X:{err_x} Y:{err_y}" + " " * 5, end="")

    except KeyboardInterrupt:
        print("\n🛑 用户强制终止")
    except Exception as e:
        print(f"\n❌ 运行时错误: {e}")
    finally:
        if 'vehicle' in locals():
            send_body_velocity(vehicle, 0, 0, 0)
            vehicle.close()
        cap.release()
        print("✅ 程序已安全退出")


if __name__ == "__main__":
    main()