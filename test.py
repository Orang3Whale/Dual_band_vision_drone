import cv2
import numpy as np
import time

# ================= ⚙️ 参数配置区域 (保持原样) =================

# --- 视觉阈值 (橙色目标) ---
# 如果环境光变化，请调节这里的阈值
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255

# --- PD 飞行控制参数 ---
MAX_SPEED = 1.0  # 最大飞行速度 (m/s)

# P (比例)
Kp_X = 0.0060
Kp_Y = 0.0060

# D (微分)
Kd_X = 0.0028
Kd_Y = 0.0028

# --- 逻辑阈值 ---
ALIGN_THRESHOLD = 100
MIN_AREA = 1000

# ========================================================

def main():
    # ---------------- Step 1: 启动摄像头 ----------------
    print("📷 正在启动摄像头 (测试模式)...")
    # 优先尝试电脑自带摄像头或 USB 摄像头 (Index 0)
    cap = cv2.VideoCapture(1) 
    
    # 如果打不开，尝试 Index 1 (外接)
    if not cap.isOpened():
        print("⚠️ Index 0 失败，尝试 Index 1...")
        cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("❌ 找不到摄像头！请检查连接。")
        return

    # 设置分辨率 (与原代码保持一致)
    cap.set(3, 640)
    cap.set(4, 480)
    
    # 动态获取中心点 (防止分辨率设置失败导致的偏差)
    ret, frame = cap.read()
    if ret:
        h, w = frame.shape[:2]
        CENTER_X, CENTER_Y = w // 2, h // 2
        print(f"✅ 摄像头就绪 | 分辨率: {w}x{h} | 中心点: ({CENTER_X}, {CENTER_Y})")
    else:
        print("❌ 无法读取画面")
        return

    # --- 变量初始化 ---
    kernel = np.ones((7, 7), np.uint8)
    prev_time = time.time()
    
    # 记录上一次的误差 (用于 D 项计算)
    last_err_x = 0.0
    last_err_y = 0.0
    
    print("\n🟢 开始测试：按 'q' 键退出")
    print("------------------------------------------------")

    while True:
        # 1. 计算 dt (模拟原代码的时间步长)
        curr_time = time.time()
        dt = curr_time - prev_time
        if dt <= 0: dt = 0.001
        prev_time = curr_time

        # 2. 读取画面
        ret, frame = cap.read()
        if not ret: break
        
        # 翻转画面 (可选：如果你觉得看着别扭，可以去掉这行)
        # frame = cv2.flip(frame, 1) 

        # ================= 核心 OpenCV 算法 =================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))
        
        # 形态学操作
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        
        # 查找轮廓
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 兼容 OpenCV 3 和 4
        contours = cnts[0] if len(cnts) == 2 else cnts[1]

        velocity_x, velocity_y = 0, 0
        err_x, err_y = 0, 0
        target_found = False

        # 绘制中心十字 (基准线)
        cv2.line(frame, (CENTER_X, 0), (CENTER_X, h), (255, 255, 255), 1)
        cv2.line(frame, (0, CENTER_Y), (w, CENTER_Y), (255, 255, 255), 1)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MIN_AREA:
                target_found = True
                ((tx, ty), radius) = cv2.minEnclosingCircle(c)
                
                # 绘制目标圆圈
                cv2.circle(frame, (int(tx), int(ty)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, (int(tx), int(ty)), 5, (0, 0, 255), -1)
                
                # 绘制误差线 (从中心到目标)
                cv2.line(frame, (CENTER_X, CENTER_Y), (int(tx), int(ty)), (0, 255, 255), 2)

                # --- 原始计算逻辑 ---
                err_x = int(tx - CENTER_X)
                err_y = int(CENTER_Y - ty) # 注意：这里是 反向计算 (Center - y)

                # PD 计算
                P_x = err_x * Kp_X
                D_x = ((err_x - last_err_x) / dt) * Kd_X
                
                P_y = err_y * Kp_Y
                D_y = ((err_y - last_err_y) / dt) * Kd_Y

                # 原代码的映射逻辑：
                # 图像 X 误差 -> 控制无人机 Y 速度 (左右平移)
                raw_vel_y = P_x + D_x
                # 图像 Y 误差 -> 控制无人机 X 速度 (前后进退)
                raw_vel_x = P_y + D_y

                last_err_x = err_x
                last_err_y = err_y

                velocity_x = np.clip(raw_vel_x, -MAX_SPEED, MAX_SPEED)
                velocity_y = np.clip(raw_vel_y, -MAX_SPEED, MAX_SPEED)
                
                # ==============================================
                # 🎨 可视化控制指令 (重点！)
                # ==============================================
                # 我们在屏幕上画箭头，代表无人机 "想要飞" 的方向
                
                # 计算箭头终点 (放大 100 倍以便观察)
                arrow_scale = 100 
                # 注意：OpenCV 画图坐标系里，Y轴向下是正，但 DroneKit 前飞(X)是正。
                # 为了直观展示 "无人机向前飞"，我们需要把箭头向上画 (Y减小)
                end_point_x = CENTER_X + int(velocity_y * arrow_scale) # Vy 控制左右
                end_point_y = CENTER_Y - int(velocity_x * arrow_scale) # Vx 控制前后 (负号是因为屏幕Y轴向下)

                cv2.arrowedLine(frame, (CENTER_X, CENTER_Y), (end_point_x, end_point_y), (0, 255, 0), 3)
                
                # 在屏幕左上角显示数值
                cv2.putText(frame, f"Err X: {err_x} -> Drone Vy: {velocity_y:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"Err Y: {err_y} -> Drone Vx: {velocity_x:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                last_err_x, last_err_y = 0, 0
        else:
            last_err_x, last_err_y = 0, 0

        # 显示画面
        cv2.imshow("Original", frame)
        cv2.imshow("Mask (Debug)", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        
        # 简单控制打印频率
        print(f"\rTarget: {'YES' if target_found else 'NO '} | ErrX:{err_x:4d} ErrY:{err_y:4d} | Cmd Vx:{velocity_x:.2f} Vy:{velocity_y:.2f}", end="")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()