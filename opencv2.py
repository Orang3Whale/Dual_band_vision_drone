import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# ================= ⚙️ 参数配置区域 ⚙️ =================

# --- 1. 飞控连接 ---
CONNECTION_STRING = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- 2. 视觉阈值 (橙色) ---
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# --- 3. 飞行控制参数 ---
MAX_SPEED_XY = 0.25      # 搜索速度
LAND_SPEED_XY = 0.1      # 降落修正速度
LAND_SPEED_Z = 0.2       # 垂直下降速度 (全程匀速)

Kp_X = 0.001
Kp_Y = 0.001

# --- 4. 逻辑阈值 ---
ALIGN_THRESHOLD = 50       # 对准阈值
HOVER_DURATION = 3.0       # 悬停时间
MIN_AREA = 1000            # 最小面积
MAX_TARGET_AREA = 215000   # 盲降面积 (屏幕的70%)

# --- 5. 系统设置 ---
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
    print(f"🐢 [无画面模式] 连接飞控: {CONNECTION_STRING} ...")
    try:
        vehicle = connect(
            CONNECTION_STRING,
            wait_ready=True,
            baud=BAUD_RATE,
            source_system=200,
            source_component=191
        )
        print(f"✅ 连接成功 | GPS: {vehicle.gps_0.fix_type} | Mode: {vehicle.mode.name}")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    # ---------------- Step 2: 启动摄像头 ----------------
    print("📷 正在后台启动摄像头...")
    cap = None
    try:
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
    except:
        pass

    if cap is None or not cap.isOpened():
        print("⚠️ 尝试备用 Index 0 ...")
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("❌ 摄像头启动失败")
        vehicle.close()
        return

    # 设置参数
    cap.set(3, 640)
    cap.set(4, 480)
    # 尝试禁用自动白平衡和自动曝光（可选，取决于摄像头支持）
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
    
    CENTER_X, CENTER_Y = 320, 240
    print("✅ 摄像头就绪 (无GUI显示)")

    # ---------------- Step 3: 逻辑循环 ----------------
    print("⏳ 等待切入 GUIDED 模式...")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)
    
    print("\n🚀 GUIDED 激活！视觉控制开始 (按 Ctrl+C 退出)\n")

    hover_start_time = None
    landing_mode = False
    kernel = np.ones((7, 7), np.uint8)
    prev_loop_time = time.time()
    loop_counter = 0

    try:
        while True:
            loop_start_time = time.time()
            loop_counter += 1

            # --- 模式监控 ---
            if vehicle.mode.name != "GUIDED":
                print("\n⚠️ 模式变更 -> 暂停控制")
                landing_mode = False
                hover_start_time = None
                while vehicle.mode.name != "GUIDED":
                    time.sleep(0.5)
                print("✅ 恢复 GUIDED 控制")

            # --- 视觉处理 ---
            ret, frame = cap.read()
            if not ret:
                print("\n❌ 视频流中断")
                break

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # --- 决策逻辑 ---
            vx, vy, vz = 0, 0, 0
            info = "Search"
            current_alt = vehicle.location.global_relative_frame.alt 

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                # 1. 盲降逻辑
                if area > MAX_TARGET_AREA:
                    vx, vy = 0, 0
                    vz = LAND_SPEED_Z
                    info = "BLIND-LAND"
                    if not landing_mode: landing_mode = True

                # 2. 正常修正
                elif area > MIN_AREA:
                    ((tx, ty), radius) = cv2.minEnclosingCircle(c)
                    err_x = int(tx - CENTER_X)
                    err_y = int(CENTER_Y - ty)

                    limit = LAND_SPEED_XY if landing_mode else MAX_SPEED_XY
                    vx = np.clip(err_y * Kp_X, -limit, limit)
                    vy = np.clip(err_x * Kp_Y, -limit, limit)

                    if abs(err_x) < ALIGN_THRESHOLD and abs(err_y) < ALIGN_THRESHOLD:
                        if not landing_mode:
                            if hover_start_time is None: hover_start_time = time.time()
                            elapsed = time.time() - hover_start_time
                            info = f"LOCK:{3.0 - elapsed:.1f}s"
                            if elapsed >= HOVER_DURATION:
                                print("\n✅ 锁定完成 -> 开始降落")
                                landing_mode = True
                        else:
                            info = "PRECISION-LAND"
                    else:
                        if not landing_mode:
                            hover_start_time = None
                            info = f"ADJUST X:{err_x} Y:{err_y}"
                        else:
                            info = "LAND-ADJUST"
                    
                    if landing_mode: vz = LAND_SPEED_Z

            else:
                # 3. 丢失目标
                if landing_mode:
                    vx, vy, vz = 0, 0, LAND_SPEED_Z
                    info = "LOST-GPS-LAND"
                else:
                    hover_start_time = None
                    info = "SEARCHING"

            # --- 触地检测 ---
            if landing_mode and current_alt < 0.2:
                print(f"\n✅ 触地 (Alt: {current_alt:.2f}m) -> 切换 LAND")
                send_body_velocity(vehicle, 0, 0, 0)
                vehicle.mode = VehicleMode("LAND")
                break

            # --- 发送指令 ---
            if loop_counter % CMD_FREQ_DIVIDER == 0:
                send_body_velocity(vehicle, vx, vy, vz)

            # --- 终端数值显示 ---
            # 使用 \r 回车符实现单行刷新，不会刷屏
            fps = 1.0 / (time.time() - prev_loop_time + 1e-6)
            prev_loop_time = time.time()
            
            # 格式化输出字符串
            status_str = f"FPS:{fps:4.1f} | Alt:{current_alt:4.1f}m | Mode:{'LANDING' if landing_mode else 'TRACKING'} | {info}"
            # 补空格防止字符残留，end="" 不换行
            print(f"\r{status_str:<60}", end="")

            # 帧率限制
            dt = time.time() - loop_start_time
            if dt < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - dt)

    except KeyboardInterrupt:
        print("\n\n🛑 用户强制停止 (Ctrl+C)")
    except Exception as e:
        print(f"\n\n❌ 错误: {e}")
    finally:
        print("正在安全退出...")
        if 'vehicle' in locals():
            send_body_velocity(vehicle, 0, 0, 0)
            vehicle.mode = VehicleMode("LAND") # 最后的保险
            vehicle.close()
        if cap:
            cap.release()
        print("✅ 完成")

if __name__ == "__main__":
    main()