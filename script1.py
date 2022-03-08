# Import packages
import time
import cv2
import os
import RPi.GPIO as GPIO
import apriltag
from PIL import Image
from numpy import asarray

# Pin Definitions
motor_pin_a = 18  # BOARD pin 18
motor_pin_b = 12  # BOARD pin 12

# ‘gstreamer_pipeline’ returns a GStreamer pipeline for capturing images from the Raspberry Pi Camera Module v2 (CSI) camera
def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def main():
    try:
        # Pin Setup
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BOARD)
        # Set both pins to LOW
        GPIO.setup(motor_pin_a, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(motor_pin_b, GPIO.OUT, initial=GPIO.LOW)
        
        # Current state of pin A (External Clock)
        curr_value_pin_a = GPIO.LOW
        # Current state of pin B (Direction)
        curr_value_pin_b = GPIO.LOW
                
        # Initialise variables
        camerafov = 62.2  # Horizontal FOV (degrees)
        camerares = [1280, 720]  # Resolution
        app = camerafov / camerares[0]  # Angle per pixel
        motorposition = 0  # Motor angle

        while True:
            ## Take picture
            # Define acquired photo file-name
            filename = 'photo.jpg'
            # Define a video capture object for the photo
            vid = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
            # Capture the video-output from the camera
            ret, frame = vid.read()
            # Write the frame into the file-name previously defined
            cv2.imwrite(filename, frame)  # Using cv2.imwrite() method to the image
            
            ## Apriltag position system
            # Load the image 'photo.jpg' from within the same directory the script is saved.
            img = Image.open('photo.jpg')
            # Convert the image to grey-scale
            imggrey = img.convert('L')
            # Convert the image into a Numpy array
            data = asarray(imggrey)
            # Define the .Detector class from within the apriltag package as 'detector'
            detector = apriltag.Detector()
            # Using the '.detect' method of the detector, detect the AprilTags in the provided image and define the output as 'result'
            result = detector.detect(data)
            # Redundancy feature in the case no AprilTag is present in the acquired photo.
            if len(result)!=0:
                # Horizontal pixel of the detected AprilTag 
                pixel = result[0][6][0]
                # Angle of the detected AprilTag from the normal
                pixelangle = (pixel - 640) * app

                # Alignment system
                # If greater than the precision
                if abs(motorposition - pixelangle) > 0.45:  
                    if motorposition > pixelangle:  # Set GPIO pin anti-clockwise
                        # Define the variable as HIGH
                        curr_value_pin_b = GPIO.HIGH
                        # Set pin 12 (direction) as HIGH
                        GPIO.output(motor_pin_b, curr_value_pin_b)
                    elif motorposition < pixelangle:  # Set GPIO pin clockwise
                        # Define the variable as LOW
                        curr_value_pin_b = GPIO.LOW
                        # Set pin 12 (direction) as LOW
                        GPIO.output(motor_pin_b, curr_value_pin_b)
                    
                    # 'For' loop determining the number of steps required
                    for i in range(1, int(round(abs(motorposition - pixelangle) / 0.9) + 1)):
                        # Set GPIO pin signal high-low
                        curr_value_pin_a ^= GPIO.HIGH
                        GPIO.output(motor_pin_a, curr_value_pin_a)
                        curr_value_pin_a ^= GPIO.LOW
                        GPIO.output(motor_pin_a, curr_value_pin_a)
                        time.sleep(0.05)

                        curr_value_pin_a ^= GPIO.HIGH
                        GPIO.output(motor_pin_a, curr_value_pin_a)
                        curr_value_pin_a ^= GPIO.LOW
                        GPIO.output(motor_pin_a, curr_value_pin_a)
                        time.sleep(0.05)

                        if curr_value_pin_b == GPIO.HIGH:  # Anti-clockwise
                            motorposition -= 0.9  # Increment the motor position variable by -0.9 for each step anti-clockwise
                        elif curr_value_pin_b == GPIO.LOW:  # Clockwise
                            motorposition += 0.9  # Increment the motor position variable by +0.9 for each step clockwise
                        print("Motor position is: ", motorposition)
            vid.release()  # After the loop release the cap object        

    except KeyboardInterrupt:
        vid.release()  # After the loop release the cap object
        # Whilst the motor position is not at the normal
        while round(motorposition) != 0:
            if motorposition > 0: # Anti-clockwise
                # Define the variable as HIGH
                curr_value_pin_b = GPIO.HIGH
                # Set pin 12 (direction) as HIGH
                GPIO.output(motor_pin_b, curr_value_pin_b)
            elif motorposition < 0: # Clockwise
                # Define the variable as LOW
                curr_value_pin_b = GPIO.LOW
                # Set pin 12 (direction) as LOW
                GPIO.output(motor_pin_b, curr_value_pin_b)
            
            # 'For' loop determining the number of steps required to return to the normal.
            for i in range(1,int(round(abs(motorposition)/0.9)+1)):
                # Set GPIO pin signal high-low
                curr_value_pin_a ^= GPIO.HIGH
                GPIO.output(motor_pin_a, curr_value_pin_a)
                curr_value_pin_a ^= GPIO.LOW
                GPIO.output(motor_pin_a, curr_value_pin_a)
                time.sleep(0.05)
                curr_value_pin_a ^= GPIO.HIGH
                GPIO.output(motor_pin_a, curr_value_pin_a)
                curr_value_pin_a ^= GPIO.LOW
                GPIO.output(motor_pin_a, curr_value_pin_a)
                time.sleep(0.05)
                
                if curr_value_pin_b == GPIO.HIGH:  # Anti-clockwise
                    motorposition -= 0.9  # Increment the motor position variable by -0.9 for each step anti-clockwise
                elif curr_value_pin_b == GPIO.LOW:  # Clockwise
                    motorposition += 0.9  # Increment the motor position variable by +0.9 for each step clockwise
                print(motorposition)
        GPIO.cleanup()  # Clean up the ports used


if __name__ == '__main__':
    main()
