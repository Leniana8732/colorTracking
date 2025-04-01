import cv2
import numpy as np
import time
import board
import busio
from adafruit_pca9685 import PCA9685


i2c = busio.I2C(board.SCL, board.SDA)  
pca = PCA9685(i2c)
pca.frequency = 50  #PWM SG90

# SG90的PWM
SERVO_CHANNEL = 0  
MIN_PULSE = 150    
MAX_PULSE = 600    

def set_servo_angle(angle):
    pulse = int(MIN_PULSE + (angle / 180.0) * (MAX_PULSE - MIN_PULSE))
    duty_cycle_value = int((pulse / 20000) * 4095 * 65535)
    pca.channels[SERVO_CHANNEL].duty_cycle = duty_cycle_value


cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: 無法開啟攝影機")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
time.sleep(1)  # 等待攝影機初始化

FRAME_WIDTH = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
if FRAME_WIDTH == 0:
    print("Warning: 無法獲取攝影機影像大小，預設使用 640")
    FRAME_WIDTH = 640  
center_x = FRAME_WIDTH // 2

# 伺服馬達初始化
current_angle = 90
set_servo_angle(current_angle)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: 讀取影像失敗")
        continue  

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        cx = x + w // 2  

        if cx < center_x - 50:
            current_angle = max(45, current_angle - 5)
        elif cx > center_x + 50:
            current_angle = min(135, current_angle + 5)
        
        set_servo_angle(current_angle)
    
    cv2.imshow("Tracking", frame)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
pca.channels[SERVO_CHANNEL].duty_cycle = 90  # 關閉馬達
