import cv2
import numpy as np

# --- 调试滑块默认值 (橙色) ---
# 如果环境太亮导致橙色发白，请在运行时拉高 S Min，拉低 V Min
h_min, s_min, v_min = 5, 120, 80
h_max, s_max, v_max = 28, 255, 255


def nothing(x): pass


def init_trackbars():
    cv2.namedWindow("Tuner")
    cv2.resizeWindow("Tuner", 400, 200)
    cv2.createTrackbar("H Min", "Tuner", h_min, 179, nothing)
    cv2.createTrackbar("S Min", "Tuner", s_min, 255, nothing)
    cv2.createTrackbar("V Min", "Tuner", v_min, 255, nothing)
    cv2.createTrackbar("H Max", "Tuner", h_max, 179, nothing)


def main():
    cap = cv2.VideoCapture(1)
    # 强制设置 720P 分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 画面中心点坐标
    CENTER_X, CENTER_Y = 640, 360

    init_trackbars()

    print("\n=== 视觉定位程序启动 ===")
    print(f"{'X 误差':^12} | {'Y 误差':^12} | {'指令建议'}")
    print("-" * 50)

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1. 获取滑块值
        h1 = cv2.getTrackbarPos("H Min", "Tuner")
        s1 = cv2.getTrackbarPos("S Min", "Tuner")
        v1 = cv2.getTrackbarPos("V Min", "Tuner")
        h2 = cv2.getTrackbarPos("H Max", "Tuner")

        # 2. 颜色提取 (HSV)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([h1, s1, v1]), np.array([h2, 255, 255]))

        # 3. 形态学处理 (关键：把空心的圆环糊成实心的饼)
        kernel = np.ones((7, 7), np.uint8)
        # 开运算去噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # 闭运算连接断裂处并填满内部空洞
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)

        # 4. 寻找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_found = False

        if contours:
            # 假设最大的橙色色块就是靶标
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            # 只有面积够大才认为是靶标 (防止误报)
            if area > 1000:
                target_found = True

                # 获取最小外接圆中心
                ((tx, ty), radius) = cv2.minEnclosingCircle(c)
                tx, ty, radius = int(tx), int(ty), int(radius)

                # ==========================================
                #       核心：计算相对于画面中心的误差
                # ==========================================

                # X轴误差：目标x - 中心x
                # 结果 < 0 : 目标在左边 (无人机需向左)
                # 结果 > 0 : 目标在右边 (无人机需向右)
                err_x = tx - CENTER_X

                # Y轴误差：中心y - 目标y (注意顺序！因为图像Y轴向下是正)
                # 结果 > 0 : 目标在上方 (无人机需向前)
                # 结果 < 0 : 目标在下方 (无人机需向后)
                err_y = CENTER_Y - ty

                # ==========================================
                #       可视化绘制
                # ==========================================

                # 1. 画出识别到的圆
                cv2.circle(frame, (tx, ty), radius, (0, 255, 255), 2)
                # 2. 画中心红点
                cv2.circle(frame, (tx, ty), 6, (0, 0, 255), -1)
                # 3. 画一根线牵着它 (从屏幕中心到目标中心)
                line_color = (0, 255, 0) if (abs(err_x) < 50 and abs(err_y) < 50) else (255, 0, 255)
                cv2.line(frame, (CENTER_X, CENTER_Y), (tx, ty), line_color, 2)

                # 4. 在画面上写出数值
                text = f"X:{err_x} Y:{err_y}"
                cv2.putText(frame, text, (tx - 60, ty - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                # 5. 终端打印
                action = ""
                if abs(err_x) < 40 and abs(err_y) < 40:
                    action = "[[悬停/降落]]"
                else:
                    action += "左飞 " if err_x < 0 else "右飞 "
                    action += "前飞 " if err_y > 0 else "后飞 "

                print(f"\r X误差: {err_x:5d} | Y误差: {err_y:5d} | {action:<10}", end="")

        if not target_found:
            print(f"\r {'正在搜索目标...':^40}", end="")

        # 画屏幕准星
        cv2.line(frame, (CENTER_X - 20, CENTER_Y), (CENTER_X + 20, CENTER_Y), (150, 150, 150), 2)
        cv2.line(frame, (CENTER_X, CENTER_Y - 20), (CENTER_X, CENTER_Y + 20), (150, 150, 150), 2)

        # 缩小显示
        disp = cv2.resize(frame, (960, 540))
        mask_disp = cv2.resize(mask, (480, 270))

        cv2.imshow("Main View", disp)
        cv2.imshow("Tuner Mask", mask_disp)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()