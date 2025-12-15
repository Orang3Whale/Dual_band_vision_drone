import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ================= ⚙️ 参数配置区域 ⚙️ =================

# --- 1. 飞控连接设置 ---
CONNECTION_STRING = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- 2. 视觉阈值 (橙色目标) ---
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# --- 3. 飞行控制参数 ---
# 水平速度限制 (m/s)
MAX_SPEED_XY = 0.25      # 搜索/对准时的最大速度
LAND_SPEED_XY = 0.12      # 降落阶段的水平修正速度 (非常温柔)

# 垂直下降速度 (m/s, 正数为向下)
# 【修改点】全程统一使用低速，不再加速冲刺
LAND_SPEED_Z = 0.2       

# PID 参数 (仅使用 P)
Kp_X = 0.001
Kp_Y = 0.001

# --- 4. 逻辑阈值 ---
ALIGN_THRESHOLD = 50       # 像素误差小于此值认为对准
HOVER_DURATION = 3.5       # 悬停保持时间 (秒)
MIN_AREA = 1000            # 最小识别面积
MAX_TARGET_AREA = 215000   # 盲降阈值 (目标占满屏幕)

# --- 5. 帧率与通信 ---
TARGET_FPS = 30
FRAME_INTERVAL = 1.0 / TARGET_FPS
CMD_FREQ_DIVIDER = 2

# 摄像头路径
CAMERA_PATH = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"

# ========================================================

def send_body_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)

def main():
    # ---------------- Step 1: 连接飞控 ----------------
    print(f"🐢 [树莓派] 连接飞控: {CONNECTION_STRING} ...")
    try:
        vehicle = connect(
            CONNECTION_STRING,
            wait_ready=True,
            baud=BAUD_RATE,
            source_system=200,
            source_component=191
        )
        print("✅ 飞控连接成功！GPS状态:", vehicle.gps_0.fix_type)
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    # ---------------- Step 2: 启动摄像头 ----------------
    print("📷 启动摄像头...")
    cap = None
    try:
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
    except:
        pass

    if cap is None or not cap.isOpened():
        print("⚠️ 尝试备用摄像头 Index 0 ...")
        cap = cv2.VideoCapture(0)

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
    
    print("🚀 视觉系统接管！")

    hover_start_time = None
    landing_mode = False
    kernel = np.ones((7, 7), np.uint8)
    prev_loop_time = time.time()
    loop_counter = 0

    try:
        while True:
            loop_start_time = time.time()
            loop_counter += 1

            # --- 安全检查 ---
            if vehicle.mode.name != "GUIDED":
                print("⚠️ 模式变更，暂停逻辑...")
                landing_mode = False
                hover_start_time = None
                while vehicle.mode.name != "GUIDED":
                    time.sleep(0.5)
                print("✅ 恢复控制！")

            # --- 读取画面 ---
            ret, frame = cap.read()
            if not ret:
                break

            # --- 图像处理 ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # --- 核心控制 ---
            velocity_x, velocity_y, velocity_z = 0, 0, 0
            target_info = "搜索中..."
            
            current_alt = vehicle.location.global_relative_frame.alt 

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                # >>> 情况 A: 目标过大 (盲降) <<<
                if area > MAX_TARGET_AREA:
                    # GPS 锁死水平位置
                    velocity_x = 0
                    velocity_y = 0
                    # 【关键】保持匀速缓慢下降，不加速
                    velocity_z = LAND_SPEED_Z
                    target_info = "🔒 GPS锁存 | 缓慢触地 (目标占满)"
                    
                    if not landing_mode:
                        landing_mode = True

                # >>> 情况 B: 目标可见 (修正) <<<
                elif area > MIN_AREA:
                    ((tx, ty), radius) = cv2.minEnclosingCircle(c)
                    err_x = int(tx - CENTER_X)
                    err_y = int(CENTER_Y - ty)

                    limit = LAND_SPEED_XY if landing_mode else MAX_SPEED_XY
                    velocity_x = np.clip(err_y * Kp_X, -limit, limit)
                    velocity_y = np.clip(err_x * Kp_Y, -limit, limit)

                    if abs(err_x) < ALIGN_THRESHOLD and abs(err_y) < ALIGN_THRESHOLD:
                        if not landing_mode:
                            if hover_start_time is None:
                                hover_start_time = time.time()
                            elapsed = time.time() - hover_start_time
                            target_info = f"锁定 | 倒计时 {HOVER_DURATION - elapsed:.1f}s"
                            
                            if elapsed >= HOVER_DURATION:
                                print("✅ 悬停完成，开始缓降！")
                                landing_mode = True
                        else:
                            target_info = "📉 精准修正中..."
                    else:
                        if not landing_mode:
                            hover_start_time = None
                            target_info = f"修正 | Err X:{err_x} Y:{err_y}"
                        else:
                            target_info = "📉 降落偏离修正..."
                    
                    if landing_mode:
                        # 【关键】目标可见时，也是匀速缓降
                        velocity_z = LAND_SPEED_Z

            # >>> 情况 C: 丢失目标 <<<
            else:
                if landing_mode:
                    # 降落中丢目标 -> GPS 锁位 + 匀速缓降
                    velocity_x = 0
                    velocity_y = 0
                    velocity_z = LAND_SPEED_Z
                    target_info = "⚠️ 丢失目标 | GPS缓降"
                else:
                    hover_start_time = None
                    target_info = "搜索目标..."

            # --- 触地保护 ---
            if landing_mode and current_alt < 0.2:
                print(f"✅ 高度 {current_alt:.2f}m -> 触地切换 LAND")
                send_body_velocity(vehicle, 0, 0, 0)
                vehicle.mode = VehicleMode("LAND")
                break

            # --- 发送指令 ---
            if loop_counter % CMD_FREQ_DIVIDER == 0:
                send_body_velocity(vehicle, velocity_x, velocity_y, velocity_z)

            # --- FPS 控制 ---
            process_duration = time.time() - loop_start_time
            if process_duration < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - process_duration)

            fps = 1.0 / (time.time() - prev_loop_time + 1e-6)
            prev_loop_time = time.time()
            print(f"\rFPS:{fps:.1f} | Alt:{current_alt:.1f}m | {target_info}" + " "*10, end="")

    except KeyboardInterrupt:
        print("\n🛑 用户终止")
    except Exception as e:
        print(f"\n❌ 运行时错误: {e}")
    finally:
        if 'vehicle' in locals():
            send_body_velocity(vehicle, 0, 0, 0)
            vehicle.mode = VehicleMode("LAND")
            vehicle.close()
        if cap:
            cap.release()

if __name__ == "__main__":
    main()