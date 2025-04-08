import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
from ultralytics import YOLO
import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw, ImageFont
import tkinter as tk
from threading import Thread

# Initialize OLED
i2c = busio.I2C(board.SCL, board.SDA)
oled = SSD1306_I2C(128, 32, i2c)
font = ImageFont.load_default()

# Tkinter UI Setup
root = tk.Tk()
root.title("Collision Avoidance System")
root.geometry("400x200")

status_label = tk.Label(root, text="Status: Waiting...", font=("Arial", 14))
status_label.pack(pady=20)

def display_message(message):
    """Displays message on both OLED and Tkinter UI."""
    oled.fill(0)  
    oled.show()
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    draw.text((10, 10), message, font=font, fill=255)
    oled.image(image)
    oled.show()
    
    # Update Tkinter UI
    root.after(500, update_status, message)

def update_status(message):
    """Updates the Tkinter label with movement instruction."""
    status_label.config(text=f"Status: {message}")

def run_detection():
    GPIO.setmode(GPIO.BCM)
    display_message("Get started")
    time.sleep(1)
    
    # Motor GPIO Setup
    in1, in2, in3, in4 = 24, 23, 22, 27
    enA, enB = 25, 26
    for pin in [in1, in2, in3, in4, enA, enB]:
        GPIO.setup(pin, GPIO.OUT)
    
    pwm_left, pwm_right = GPIO.PWM(enA, 1000), GPIO.PWM(enB, 1000)
    pwm_left.start(0)
    pwm_right.start(0)
    
    def move_forward():
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
        pwm_left.ChangeDutyCycle(40)
        pwm_right.ChangeDutyCycle(40)
    
    def stop():
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        pwm_left.ChangeDutyCycle(0)
        pwm_right.ChangeDutyCycle(0)
    
    def turn_left(a, b):
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.HIGH)
        GPIO.output(in4, GPIO.LOW)
        pwm_left.ChangeDutyCycle(a)
        pwm_right.ChangeDutyCycle(b)
    
    def turn_right(a, b):
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        GPIO.output(in3, GPIO.LOW)
        GPIO.output(in4, GPIO.LOW)
        pwm_right.ChangeDutyCycle(a)
        pwm_left.ChangeDutyCycle(b)
    
    model = YOLO("yolov8n.pt")
    FOCAL_LENGTH, KNOWN_WIDTH, SAFE_DISTANCE = 700, 50, 500
    LANE_LEFT, LANE_RIGHT = 0, 1000
    A = False
    
    def estimate_distance(bbox_width):
        return float("inf") if bbox_width == 0 else (KNOWN_WIDTH * FOCAL_LENGTH) / bbox_width
    
    def calculate_angle(x1, y1, x2, y2):
        return (x1 + x2) // 2
    
    def get_movement_action(angle, distance):
        if distance < SAFE_DISTANCE:
            if angle < 200:
                return "Move Right"
            if angle > 400:
                return "Move Left"
            return "BRAKE! STOP"
        return "Keep Moving"
       
    cap = cv2.VideoCapture(0)
    cnt=2
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            if(cnt>0):
                cnt-=1
                continue
            s = set()
            results = model(frame)
            
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    bbox_width = x2 - x1
                    object_center_x = (x1 + x2) // 2

                    if LANE_LEFT <= object_center_x <= LANE_RIGHT:
                        distance = estimate_distance(bbox_width)
                        angle = calculate_angle(x1, y1, x2, y2)
                        action = get_movement_action(angle, distance)
                        color = (0, 255, 0) if distance > SAFE_DISTANCE else (0, 0, 255)

                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, f"Dist: {int(distance)} cm", (x1, y1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        cv2.putText(frame, action, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        s.add(action)
                        
            if ("Move Right" in s and "Move Left" in s) or "BRAKE! STOP" in s:
                action = "BRAKE! STOP"
            elif "Move Right" in s:
                action = "Move Right"
            elif "Move Left" in s:
                action = "Move Left"
            else:
                action = "Keep Moving"
            
            cv2.putText(frame, action, (500, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            print(action)
            display_message(action)
            time.sleep(0.5)
            
            if A == False:
                if action == "Move Right":
                    move_forward()
                    time.sleep(0.8)
                    turn_right(25, 45)
                    time.sleep(0.5)
                    move_forward()
                    time.sleep(0.3)
                    turn_left(25, 45)
                    time.sleep(0.5)
                    move_forward()
                    time.sleep(0.8)
                    stop()
                    A = True
            
                elif action == "Move Left":
                    move_forward()
                    time.sleep(0.8)
                    turn_left(25, 45)
                    time.sleep(0.5)
                    move_forward()
                    time.sleep(0.2)
                    turn_right(25, 45)
                    time.sleep(0.6)
                    move_forward()
                    time.sleep(0.8)
                    stop()
                    A = True
            
                elif action == "BRAKE! STOP":
                    move_forward()
                    time.sleep(1)
                    stop()
                    time.sleep(0.5)
                    A = True
                        
            cv2.imshow("Detection & Warning", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        print("Interrupted by user")
    
    finally:
        stop()
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        oled.fill(0)  
        oled.show()

def start_program():
    detection_thread = Thread(target=run_detection)
    detection_thread.start()

start_button = tk.Button(root, text="Start", command=start_program, font=("Arial", 14))
start_button.pack(pady=20)

root.mainloop()
    
