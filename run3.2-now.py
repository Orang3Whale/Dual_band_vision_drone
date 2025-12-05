import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ================= ⚙️ 参数配置区域 ⚙️ =================

# --- 1. 飞控连接设置 ---
# 使用 USB 连接
CONNECTION_STRING = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- 2. 视觉阈值 (橙色目标) ---
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# --- 3. 飞行控制参数 (PID) ---
MAX_SPEED = 0.25  # 最大飞行速度 (m/s)
Kp_X = 0.001      # X轴比例系数
Kp_Y = 0.001      # Y轴比例系数

# --- 4. 逻辑阈值 ---
ALIGN_THRESHOLD = 40  # 像素误差小于此值认为对准
HOVER_DURATION = 4.0  # 悬停保持时间 (秒)
MIN_AREA = 1000       # 最小识别面积

# --- 5. 帧率限制 ---
TARGET_FPS = 30       # 目标帧率
FRAME_INTERVAL = 1.0 / TARGET_FPS

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
    print(f"�� [树莓派] 正在通过 USB 连接飞控: {CONNECTION_STRING} ...")
    try:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE)
        print("✅ 飞控连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    # ---------------- Step 2: 智能启动摄像头 (自动切换) ----------------
    print("�� 正在启动摄像头 (优先尝试 USB摄像头 Index 1)...")
    
    # 1. 先尝试打开 Index 1
    cap = cv2.VideoCapture(1)
    
    # 2. 检查是否成功，如果不成功，尝试 Index 0
    if not cap.isOpened():
        print("⚠️ 无法打开摄像头 1，正在尝试备用摄像头 0 ...")
        cap = cv2.VideoCapture(0)

    # 3. 再次检查，如果还是打不开，那就彻底没戏了
    if not cap.isOpened():
        print("❌ 致命错误：找不到任何可用的摄像头！请检查连接。")
        vehicle.close()
        return

    # 设置分辨率
    width = 640
    height = 480
    cap.set(3, width)
    cap.set(4, height)
    cap.set(cv2.CAP_PROP_FPS, 30)

    CENTER_X = width // 2
    CENTER_Y = height // 2

    print(f"✅ 摄像头就绪 ({width}x{height})")

    # ---------------- Step 3: 等待 GUIDED 模式 ----------------
    print("⏳ 等待切入 GUIDED 模式以开始控制...")
    
    last_print_time = 0
    while vehicle.mode.name != "GUIDED":
        if time.time() - last_print_time > 2:
            print(f"当前模式: {vehicle.mode.name} (等待 GUIDED)...")
            last_print_time = time.time()
        time.sleep(0.5)

    print("�� GUIDED 模式激活！视觉系统接管控制！")

    hover_start_time = None
    kernel = np.ones((7, 7), np.uint8)
    prev_loop_time = time.time()

    try:
        while True:
            loop_start_time = time.time()

            # --- 安全检查 ---
            if vehicle.mode.name != "GUIDED":
                print("⚠️ 模式变更，暂停控制 (等待切回 GUIDED)...")
                send_body_velocity(vehicle, 0, 0, 0)
                while vehicle.mode.name != "GUIDED":
                    time.sleep(1)
                print("✅ 恢复控制！")
                hover_start_time = None

            # --- 读取画面 ---
            ret, frame = cap.read()
            if not ret:
                print("❌ 摄像头画面丢失 (掉线)")
                break

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
                area = cv2.contourArea(c)

                if area > MIN_AREA:
                    ((tx, ty), radius) = cv2.minEnclosingCircle(c)
                    err_x = int(tx - CENTER_X)
                    err_y = int(CENTER_Y - ty)

                    # PID 计算
                    velocity_x = np.clip(err_y * Kp_X, -MAX_SPEED, MAX_SPEED)
                    velocity_y = np.clip(err_x * Kp_Y, -MAX_SPEED, MAX_SPEED)

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
                        target_info = f"修正 | Vx:{velocity_x:.2f} Vy:{velocity_y:.2f}"
                else:
                    hover_start_time = None
                    target_info = f"噪点过滤 ({int(area)})"
            else:
                hover_start_time = None
                target_info = "丢失目标"

            send_body_velocity(vehicle, velocity_x, velocity_y, 0)

            # ================= FPS 限制与显示 =================
            
            process_duration = time.time() - loop_start_time
            if process_duration < FRAME_INTERVAL:
                time.sleep(FRAME_INTERVAL - process_duration)

            curr_time = time.time()
            actual_loop_time = curr_time - prev_loop_time
            prev_loop_time = curr_time
            fps = 1.0 / actual_loop_time if actual_loop_time > 0 else 0

            print(f"\rFPS: {fps:.1f} | {target_info} | Err X:{err_x} Y:{err_y}" + " " * 5, end="")

    except KeyboardInterrupt:
        print("\n�� 用户强制终止")
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