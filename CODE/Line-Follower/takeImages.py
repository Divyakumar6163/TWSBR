from picamera2 import Picamera2
import time

# Initialize the camera
picam2 = Picamera2()

# Start the camera
picam2.start()

try:
    while True:
        # Capture the image and overwrite it with the same name
        picam2.capture_file("test.jpg")
        print("Image captured and saved as 'test.jpg'")
        
        # Wait for 500ms before capturing the next image
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Stopping the image capture loop.")
finally:
    # Stop the camera when exiting
    picam2.stop()