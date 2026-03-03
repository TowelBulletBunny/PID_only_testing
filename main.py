import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import time
import csv

# --- MOTOR SETUP ---
GPIO.setmode(GPIO.BCM)
ENA, IN1, IN2 = 12, 23, 24
ENB, IN3, IN4 = 13, 17, 27

GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
pwmA = GPIO.PWM(ENA, 1000); pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0); pwmB.start(0)

# --- GLOBAL PID VARIABLES ---
last_error = 0
integral = 0

# --- CAMERA SETUP ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (160, 120)})
picam2.configure(config)
picam2.start()

def get_line_error():
    """Captures a frame and returns horizontal error or 'FINISH'."""
    frame = picam2.capture_array()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    roi = gray[75:115, 0:160]  
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    
    # This turns black tape into WHITE pixels
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    M = cv2.moments(thresh)
    pixel_count = M['m00'] / 255  

    if pixel_count > 10000:
        return "FINISH"

    if pixel_count > 400:
        cx = int(M['m10'] / M['m00']) 
        return cx - 80  
    else:
        return None

def move_robot(error):
    global last_error, integral
    
    # PID Constants - Tweak these based on your "Balanced" table
    Kp, Ki, Kd = 0.5, 0.01, 0.2
    base_speed = 40 

    derivative = error - last_error
    integral += error
    
    # Anti-windup and center-reset
    integral = max(-500, min(500, integral))
    if (error > 0 and last_error < 0) or (error < 0 and last_error > 0):
        integral = 0

    steering = (error * Kp) + (integral * Ki) + (derivative * Kd)
    last_error = error 

    # STANDARD PID: Forward only
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.HIGH)

    # Calculate power and clamp 0-100
    left_pwr = max(0, min(100, base_speed + steering))
    right_pwr = max(0, min(100, base_speed - steering))

    pwmA.ChangeDutyCycle(left_pwr)
    pwmB.ChangeDutyCycle(right_pwr)
    
    return left_pwr, right_pwr

def stop_motors():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwmA.ChangeDutyCycle(0); pwmB.ChangeDutyCycle(0)

# --- PRE-START SETUP ---
# Create/Open a CSV file for logging
log_file = open("robot_tuning_log.csv", mode="w", newline="")
log_writer = csv.writer(log_file)
log_writer.writerow(["Time", "Error", "L_Motor", "R_Motor"])

print("Systems Online.")
input(">>> Place robot on track and press ENTER to start! <<<")

# --- MAIN LOOP ---
try:
    start_time = time.time()
    while True:
        result = get_line_error()
        
        # Create the dashboard (180px high to fit all data)
        status_img = np.zeros((180, 400, 3), dtype="uint8")

        if result == "FINISH":
            print("Finish line detected!")
            stop_motors()
            break 
        
        elif result is not None:
            # Memory logic for search
            if abs(result) > 20: 
                last_error = result
            
            l_pwr, r_pwr = move_robot(result)
            
            # --- DASHBOARD LOGIC ---
            # If motor is 0 (saturated), text turns RED
            l_clr = (0, 0, 255) if l_pwr == 0 else (0, 255, 0)
            r_clr = (0, 0, 255) if r_pwr == 0 else (0, 255, 0)

            cv2.putText(status_img, f"Error: {result}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(status_img, f"L-Motor: {l_pwr:.1f}%", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, l_clr, 2)
            cv2.putText(status_img, f"R-Motor: {r_pwr:.1f}%", (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, r_clr, 2)
            
            # --- CSV LOGGING ---
            elapsed = time.time() - start_time
            log_writer.writerow([round(elapsed, 2), result, l_pwr, r_pwr])
            
        else:
            stop_motors() # Stop and wait for search logic if desired
            cv2.putText(status_img, "LOST - STOPPED", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow("Robot Dashboard", status_img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'): break

except KeyboardInterrupt:
    print("\nUser stopped the robot.")
finally:
    log_file.close() # Save the CSV
    print("Log saved to 'robot_tuning_log.csv'")
    stop_motors()
    pwmA.stop(); pwmB.stop()
    picam2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
