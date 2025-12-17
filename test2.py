import cv2
import numpy as np

def empty(a):
    pass

# --- 1. 初始化摄像头 ---
print("正在启动摄像头...")
cap = cv2.VideoCapture(1) # 如果打不开试试 1

# 如果是树莓派，建议降低分辨率以提高流畅度
cap.set(3, 640)
cap.set(4, 480)

# --- 2. 创建滑块窗口 ---
cv2.namedWindow("HSV Tuner")
cv2.resizeWindow("HSV Tuner", 640, 240)

# 创建 6 个滑块：H(色调), S(饱和度), V(亮度) 的最小值和最大值
# 默认值设为宽泛范围，方便一开始能看到东西
cv2.createTrackbar("H Min", "HSV Tuner", 0,   179, empty)
cv2.createTrackbar("H Max", "HSV Tuner", 179, 179, empty)
cv2.createTrackbar("S Min", "HSV Tuner", 0,   255, empty)
cv2.createTrackbar("S Max", "HSV Tuner", 255, 255, empty)
cv2.createTrackbar("V Min", "HSV Tuner", 0,   255, empty)
cv2.createTrackbar("V Max", "HSV Tuner", 255, 255, empty)

while True:
    ret, frame = cap.read()
    if not ret: break

    # 转换到 HSV 空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 实时读取滑块的值
    h_min = cv2.getTrackbarPos("H Min", "HSV Tuner")
    h_max = cv2.getTrackbarPos("H Max", "HSV Tuner")
    s_min = cv2.getTrackbarPos("S Min", "HSV Tuner")
    s_max = cv2.getTrackbarPos("S Max", "HSV Tuner")
    v_min = cv2.getTrackbarPos("V Min", "HSV Tuner")
    v_max = cv2.getTrackbarPos("V Max", "HSV Tuner")

    # 创建掩膜 (Mask)
    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)

    # 显示结果
    # 原始画面 + 识别结果 (Mask)
    # Mask 中：白色 = 识别到的东西，黑色 = 忽略的东西
    cv2.imshow("Original", frame)
    cv2.imshow("Mask (Result)", mask)

    # 按 'q' 键打印当前数值并退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("\n" + "="*30)
        print("✅ 最终调试结果 (请复制到飞控代码):")
        print(f"H_MIN, S_MIN, V_MIN = {h_min}, {s_min}, {v_min}")
        print(f"H_MAX, S_MAX, V_MAX = {h_max}, {s_max}, {v_max}")
        print("="*30 + "\n")
        break

cap.release()
cv2.destroyAllWindows()