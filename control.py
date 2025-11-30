import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ================= 🐢 慢速版参数设置 🐢 =================

# 1. 连接 (根据你的硬件修改)
CONNECTION_STRING = '/dev/ttyACM0'
BAUD_RATE = 57600

# 2. 视觉阈值 (橙色)
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# 3. 速度控制 (关键修改区域)
# -----------------------------------------------------
# 最大速度限制 (米/秒)
# 0.2 m/s 非常慢，像乌龟爬一样，给你充足的反应时间
MAX_SPEED = 0.25

# PID 比例系数 (Kp)
# 值越小，无人机修正越温柔；值越大，修正越猛烈
# 0.001 代表：误差 100 像素 -> 速度仅为 0.1 m/s
Kp_X = 0.001
Kp_Y = 0.001
# -----------------------------------------------------

# 4. 逻辑阈值
ALIGN_THRESHOLD = 40  # 放宽一点对准范围 (40像素)，因为速度慢，太严格很难对准
HOVER_DURATION = 4.0  # 悬停确认时间 (秒)


# =========================================================

def send_body_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    # 发送速度指令 (NED坐标系: x前, y右, z下)
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,  # 只启用速度控制
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)


def main():
    print(f"🐢 启动慢速控制模式，连接飞控: {CONNECTION_STRING} ...")
    try:
        vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=BAUD_RATE)
        print("✅ 飞控连接成功！")
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return

    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)
    cap.set(4, 720)
    CENTER_X, CENTER_Y = 640, 360

    print("\n------------------------------------------------")
    print(f"当前限速: {MAX_SPEED} m/s (非常安全)")
    print("请手动起飞到 2-3米 高度，悬停在靶标附近。")
    print("确认安全后，按 Enter 键切换 GUIDED 模式开始接管...")
    print("------------------------------------------------\n")
    input()

    # 切换模式
    if vehicle.mode.name != "GUIDED":
        print("正在切换到 GUIDED 模式...")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

    hover_start_time = None

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            # --- 视觉处理 ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))

            # 形态学去噪 (填补圆心空洞)
            kernel = np.ones((7, 7), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            velocity_x = 0
            velocity_y = 0
            target_found = False

            if contours:
                c = max(contours, key=cv2.contourArea)
                # 面积阈值，防止把地上的小碎石看成靶标
                if cv2.contourArea(c) > 1000:
                    target_found = True
                    ((tx, ty), radius) = cv2.minEnclosingCircle(c)
                    tx, ty = int(tx), int(ty)

                    # --- 误差计算 ---
                    err_x = tx - CENTER_X
                    err_y = CENTER_Y - ty  # 图片Y向下为正，无人机X向前为正

                    # --- 速度计算 (P控制 + 限速) ---
                    # 1. 原始计算: 误差 * 灵敏度
                    raw_vx = err_y * Kp_X
                    raw_vy = err_x * Kp_Y

                    # 2. 安全限幅: 无论误差多大，绝不超过 MAX_SPEED (0.2 m/s)
                    velocity_x = np.clip(raw_vx, -MAX_SPEED, MAX_SPEED)
                    velocity_y = np.clip(raw_vy, -MAX_SPEED, MAX_SPEED)

                    # --- 悬停判定逻辑 ---
                    is_aligned = abs(err_x) < ALIGN_THRESHOLD and abs(err_y) < ALIGN_THRESHOLD

                    if is_aligned:
                        if hover_start_time is None:
                            hover_start_time = time.time()

                        elapsed = time.time() - hover_start_time
                        remaining = HOVER_DURATION - elapsed

                        # 终端显示倒计时
                        print(f"\r[✨ 锁定中] 剩余 {remaining:.1f}s | 速度 x:{velocity_x:.2f} y:{velocity_y:.2f}  ",
                              end="")

                        # 画面显示
                        cv2.circle(frame, (tx, ty), int(radius), (0, 255, 0), 3)
                        cv2.putText(frame, f"LOCK: {elapsed:.1f}s", (tx, ty - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                    (0, 255, 0), 2)

                        # 时间到 -> 降落
                        if elapsed >= HOVER_DURATION:
                            print("\n\n✅ 悬停稳定！开始自动降落 (LAND)...")
                            # 先刹车，停稳
                            send_body_velocity(vehicle, 0, 0, 0)
                            time.sleep(0.5)
                            # 切换降落模式
                            vehicle.mode = VehicleMode("LAND")
                            break
                    else:
                        # 误差过大，重置计时
                        if hover_start_time is not None:
                            print("\n⚠️ 偏离目标，重新调整...")
                        hover_start_time = None

                        # 显示正在修正
                        print(f"\r[🐢 慢速修正] 误差 X:{err_x} Y:{err_y} | 速度 {velocity_x:.2f}, {velocity_y:.2f}   ",
                              end="")
                        cv2.circle(frame, (tx, ty), int(radius), (0, 255, 255), 2)
                        cv2.line(frame, (CENTER_X, CENTER_Y), (tx, ty), (255, 0, 0), 1)

            else:
                # 丢失目标 -> 悬停
                hover_start_time = None
                velocity_x = 0
                velocity_y = 0
                print(f"\r[🔍 搜索目标] 保持悬停...                        ", end="")

            # 发送指令 (Z轴给0，保持定高)
            if vehicle.mode.name == "GUIDED":
                send_body_velocity(vehicle, velocity_x, velocity_y, 0)

            # 显示画面
            cv2.imshow("Drone Eyes", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n🛑 紧急停止！发送刹车指令...")
        send_body_velocity(vehicle, 0, 0, 0)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        vehicle.close()


if __name__ == "__main__":
    main()