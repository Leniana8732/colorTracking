import time
import board
import busio
import cv2
import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# -------------------------------
# 1. 初始化 PCA9685 與 SG90 伺服馬達
# -------------------------------

# 初始化 I2C 介面
i2c = busio.I2C(board.SCL, board.SDA)

# 初始化 PCA9685，設定 PWM 頻率為 50Hz (SG90 標準)
pca = PCA9685(i2c)
pca.frequency = 50

# 建立連接到 PCA9685 第 0 通道的 SG90 伺服馬達物件
servo0 = servo.Servo(pca.channels[0])

deadzone = 3

# 定義設定伺服馬達角度的函數，加入死區檢查
def set_servo_angle(target_angle):
    global current_angle
    # 如果目標角度與目前角度差異小於死區閾值，則不更新
    if abs(target_angle - current_angle) < deadzone:
        return
    # 更新角度
    current_angle = target_angle
    servo0.angle = current_angle
    time.sleep(0.5)

# 設定初始角度= 90°(置中)
set_servo_angle(90)

# -------------------------------
# 2. 初始化 USB 攝影機 (PW313)
# -------------------------------

#開啟攝影機
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   #設定解析度 640 x 480
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# -------------------------------
# 3. 物體追蹤與伺服馬達控制
# -------------------------------

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 將 BGR 影像轉換為 HSV 色彩空間
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 定義綠色範圍 
    #Upper HSV:  (67, 255, 255)
    #Lower HSV:  (40, 79, 32)
    lower = np.array([40, 79, 32])
    upper = np.array([67, 255, 255])
    
    # 篩出綠色
    mask = cv2.inRange(hsv, lower, upper)
    
    # 尋找輪廓
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # 取最大輪廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        # 過濾雜訊
        if area > 500:
            # 取得物體邊界，計算中心點
            x, y, w, h = cv2.boundingRect(largest_contour)
            cx, cy = x + w // 2, y + h // 2
            
            # 畫出矩形與中心點 (視覺化)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
            
            # 根據物體中心點位置調整伺服馬達角度：
            # 若物體位於畫面左側 (小於 1/3 寬度)，設定角度為 45°
            if cx < frame.shape[1] // 3:
                target_angle = 45
                print("turn left")
            # 若物體位於畫面右側 (大於 2/3 寬度)，設定角度為 135°
            elif cx > 2 * frame.shape[1] // 3:
                target_angle = 135
                print("turn right")
            # 否則，物體位於中間，設定角度為 90°
            else:
                target_angle = 90
                print("middle")
            
            # 控制伺服馬達轉動到目標角度
            set_servo_angle(target_angle)

    # 顯示原始影像與遮罩
    cv2.imshow("Tracking", frame)
    cv2.imshow("Mask", mask)
    
    # 按 'q' 鍵退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 釋放攝影機與關閉視窗
cap.release()
cv2.destroyAllWindows()
