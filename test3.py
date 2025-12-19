import cv2
import numpy as np

# ================= ⚙️ 参数配置 (保持与主程序一致) ⚙️ =================
# 1. 视觉阈值 (橙色) - 如果主程序改了这里也要改
H_MIN, S_MIN, V_MIN = 9 , 130, 172
H_MAX, S_MAX, V_MAX = 179, 255, 255

# 2. 摄像头设置
CAMERA_INDEX = 1    # 电脑自带一般是0，外接USB一般是1或0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
# =================================================================

def main():
    # 启动摄像头
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    # 强制设置分辨率，确保计算的百分比与飞控代码一致
    cap.set(3, FRAME_WIDTH)
    cap.set(4, FRAME_HEIGHT)
    
    # 计算画面总像素 (分母)
    TOTAL_PIXELS = FRAME_WIDTH * FRAME_HEIGHT
    
    print(f"📷 摄像头已启动 | 分辨率: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"📏 总像素数: {TOTAL_PIXELS}")
    print("👉 请移动摄像头模拟降落，观察 'Ratio' 的数值变化")
    print("⌨️  按 'q' 键退出")

    kernel = np.ones((5, 5), np.uint8)

    while True:
        ret, frame = cap.read(0)
        if not ret:
            print("❌ 无法读取画面，请检查摄像头连接")
            break

        # --- 图像处理 (与主程序完全一致) ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([H_MIN, S_MIN, V_MIN]), np.array([H_MAX, S_MAX, V_MAX]))
        
        # 形态学去噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # --- 显示逻辑 ---
        if contours:
            # 找到最大轮廓
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            # 🔢 核心计算：面积占比
            ratio = area / TOTAL_PIXELS
            percentage = ratio * 100
            
            # 绘制绿色轮廓
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
            
            # 绘制数据面板
            # 背景黑框，让字看得清楚
            cv2.rectangle(frame, (0, 0), (250, 80), (0, 0, 0), -1) 
            
            # 显示面积 (像素)
            cv2.putText(frame, f"Area: {int(area)} px", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示占比 (%) - 这是你要的数据！
            color = (0, 255, 255) # 黄色
            if percentage > 30: color = (0, 0, 255) # 超过30%变红(模拟触发)
            
            cv2.putText(frame, f"Ratio: {percentage:.2f}%", (10, 65), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        else:
            # 没找到目标
            cv2.putText(frame, "NO TARGET", (10, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 显示画面
        cv2.imshow('Landing Area Calibration Tool', frame)
        # cv2.imshow('Mask', mask) # 如果想看二值化效果，取消这行注释

        # 退出检测
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()