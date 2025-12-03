import cv2
import numpy as np
import time

# ================= ⚡️ 极致优化设置 ⚡️ =================

# 1. 目标帧率 (锁定 20 FPS，足够控制且省电)
TARGET_FPS = 20
TARGET_CYCLE_TIME = 1.0 / TARGET_FPS

# 2. 分辨率设置
# ⚠️ 关键优化：使用 320x240 代替 720P
# 像素点减少 12 倍，速度飞快，CPU 占用极低
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CENTER_X = FRAME_WIDTH // 2
CENTER_Y = FRAME_HEIGHT // 2

# 3. 颜色阈值 (由于没有界面滑块，需在此手动填写)
# 这里填入你在 PC 上调试好的数值
H_MIN, S_MIN, V_MIN = 5, 120, 80
H_MAX, S_MAX, V_MAX = 28, 255, 255


# =======================================================

def main():
    print(f"🚀 启动低功耗模式 | 分辨率: {FRAME_WIDTH}x{FRAME_HEIGHT} | 限制: {TARGET_FPS} FPS")

    # 1. 初始化摄像头 (优先尝试 index 0)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("⚠️ 尝试自动搜索摄像头...")
        cap = cv2.VideoCapture(-1)

    if not cap.isOpened():
        print("❌ 无法打开摄像头，请检查排线或 Legacy 设置")
        return

    # 设置低分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # 预分配一个小核 (3x3 比 7x7 更快)
    kernel = np.ones((3, 3), np.uint8)

    print("-" * 50)
    print(f"{'FPS':^6} | {'X 误差':^8} | {'Y 误差':^8} | {'状态'}")
    print("-" * 50)

    try:
        while True:
            # 记录起始时间
            loop_start = time.time()

            ret, frame = cap.read()
            if not ret:
                print("\r❌ 摄像头断开", end="")
                time.sleep(0.5)
                continue

            # ================= 核心视觉处理 (无绘图) =================

            # 1. 颜色转换
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 2. 二值化
            mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))

            # 3. 极速去噪 (简化版形态学)
            # 只做一次腐蚀和两次膨胀，比原来的 Open+Close 快很多
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)

            # 4. 轮廓查找
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 数据初始化
            target_status = "Search"
            err_x = 0
            err_y = 0

            if contours:
                c = max(contours, key=cv2.contourArea)
                # 面积阈值按比例缩小 (因为分辨率变小了)
                if cv2.contourArea(c) > 200:
                    # 使用矩计算中心 (比最小外接圆函数快)
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        err_x = cx - CENTER_X
                        err_y = CENTER_Y - cy
                        target_status = "Locked"

            # ================= 帧率控制与显示 =================

            # 1. 计算处理耗时
            process_duration = time.time() - loop_start

            # 2. 智能休眠 (补足时间差)
            sleep_time = TARGET_CYCLE_TIME - process_duration
            if sleep_time > 0:
                time.sleep(sleep_time)

            # 3. 计算真实 FPS
            real_fps = 1.0 / (time.time() - loop_start)

            # 4. 终端输出 (使用 \r 刷新同一行)
            if target_status == "Locked":
                # 绿色显示数值 (如果终端支持)
                print(f"\r {real_fps:4.1f}  | {err_x:8d} | {err_y:8d} | 🎯 锁定", end="")
            else:
                print(f"\r {real_fps:4.1f}  | {'---':^8} | {'---':^8} | 🔍 搜索", end="")

    except KeyboardInterrupt:
        print("\n🛑 程序停止")
    finally:
        cap.release()
        print("资源已释放")


if __name__ == "__main__":
    main()