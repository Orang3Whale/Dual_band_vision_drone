import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ================= ⚙️ 参数配置区域 ⚙️ =================

# --- 1. 飞控连接设置 ---
# [关键修改] 树莓派通过 GPIO 杜邦线连接飞控 (TELEM 2)
# 通常设备名为 /dev/ttyAMA0 或 /dev/serial
CONNECTION_STRING = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- 2. 视觉阈值 (橙色目标) ---
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# --- 3. 飞行控制参数 (PID) ---
MAX_SPEED = 0.25  # 最大飞行速度 (m/s)
Kp_X = 0.001  # X轴比例系数 (左右移动)
Kp_Y = 0.001  # Y轴比例系数 (前后移动)

# --- 4. 逻辑阈值 ---
ALIGN_THRESHOLD = 40  # 像素误差小于此值认为对准
HOVER_DURATION = 4.0  # 悬停保持时间 (秒)
MIN_AREA = 1000  # 最小识别面积


# ========================================================

def send_body_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """
    发送机体坐标系下的速度指令 (单位: m/s)
    velocity_x: 前(+)后(-)
    velocity_y: 右(+)左(-)
    velocity_z: 下(+)上(-)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # 掩码: 忽略位置，只控制速度
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)


def main():
    # ---------------- Step 1: 连接飞控 ----------------
    print(f"🐢 [树莓派] 正在通过 GPIO 连接飞控: {CONNECTION_STRING} ...")
    try:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE)
        print("✅ 飞控连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        print("💡 提示: 请检查 RX/TX 线序是否接反，以及飞控 SERIAL2_BAUD 是否设为 57")
        return

    # ---------------- Step 2: 初始化摄像头 ----------------
    print("📷 正在启动摄像头...")
    cap = cv2.VideoCapture(1)

    # 设置分辨率 640x480
    width = 640
    height = 480
    cap.set(3, width)
    cap.set(4, height)

    CENTER_X = width // 2
    CENTER_Y = height // 2

    if not cap.isOpened():
        print("❌ 无法打开摄像头，请检查连接")
        vehicle.close()
        return

    print(f"✅ 摄像头就绪 ({width}x{height})")

    # ---------------- Step 3: 等待 GUIDED 模式 ----------------
    print("⏳ 等待切入 GUIDED 模式以开始控制...")

    while vehicle.mode.name != "GUIDED":
        # 每秒刷新一次状态
        print(f"\r当前模式: {vehicle.mode.name} (等待 GUIDED)...", end="")
        time.sleep(1)

    print("\n\n🚀 GUIDED 模式激活！视觉系统接管控制！")

    # 初始化变量
    hover_start_time = None
    prev_time = time.time()
    kernel = np.ones((7, 7), np.uint8)

    try:
        while True:
            # --- 安全检查: 模式是否被切走 ---
            if vehicle.mode.name != "GUIDED":
                print("\n⚠️ 模式变更，暂停控制 (等待切回 GUIDED)...")
                send_body_velocity(vehicle, 0, 0, 0)  # 立即刹车
                while vehicle.mode.name != "GUIDED":
                    time.sleep(1)
                print("\n✅ 恢复控制！")
                hover_start_time = None
                prev_time = time.time()

            # --- 读取画面 ---
            ret, frame = cap.read()
            if not ret:
                print("\n❌ 摄像头画面丢失")
                break

            # ================= 核心 OpenCV 算法 =================

            # 1. 色彩空间转换
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 2. 颜色二值化
            mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))

            # 3. 形态学操作
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

            # 4. 轮廓查找
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # ================= 目标分析与控制计算 =================

            velocity_x = 0
            velocity_y = 0
            target_info = "搜索中..."
            err_x = 0
            err_y = 0

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                if area > MIN_AREA:
                    ((tx, ty), radius) = cv2.minEnclosingCircle(c)

                    # 计算误差
                    err_x = int(tx - CENTER_X)
                    err_y = int(CENTER_Y - ty)

                    # PID 控制量计算
                    # 图像上方(小Y) -> 无人机前方(+X)
                    # 图像右方(大X) -> 无人机右方(+Y)
                    raw_vx = err_y * Kp_X
                    raw_vy = err_x * Kp_Y

                    # 速度限幅
                    velocity_x = np.clip(raw_vx, -MAX_SPEED, MAX_SPEED)
                    velocity_y = np.clip(raw_vy, -MAX_SPEED, MAX_SPEED)

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

            # 发送控制指令
            send_body_velocity(vehicle, velocity_x, velocity_y, 0)

            # ================= FPS 计算与实时显示 =================

            curr_time = time.time()
            exec_time = curr_time - prev_time
            prev_time = curr_time
            fps = 1 / exec_time if exec_time > 0 else 0

            # 终端打印
            print(f"\r📸 FPS: {fps:.1f} | {target_info} | Err X:{err_x} Y:{err_y}" + " " * 10, end="")

            # 防止 CPU 占用过高
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n\n🛑 用户强制终止")

    except Exception as e:
        print(f"\n❌ 运行时错误: {e}")

    finally:
        print("正在清理资源...")
        if 'vehicle' in locals():
            send_body_velocity(vehicle, 0, 0, 0)
            vehicle.close()
        cap.release()
        print("✅ 程序已安全退出")

if __name__ == "__main__":
    main()
