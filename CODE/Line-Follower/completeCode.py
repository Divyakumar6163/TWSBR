from picamera2 import Picamera2
import time
import cv2 as cv
import numpy as np

# Configuration settings
class tconf:
    white_min = 10
    white_max = 70
    threshold_min = 50
    threshold_max = 200
    th_iterations = 10
    turn_angle = 20
    shift_max = 30
    shift_step = 5
    turn_step = 10
    straight_run = 0.1

# Adjust the threshold dynamically
def balance_pic(image):
    global T
    T = 100
    ret = None
    direction = 0
    for i in range(tconf.th_iterations):
        _, gray = cv.threshold(image, T, 255, cv.THRESH_BINARY_INV)
        crop = Roi.crop_roi(gray)
        nwh = cv.countNonZero(crop)
        perc = int(100 * nwh / Roi.get_area())

        if perc > tconf.white_max:
            if T > tconf.threshold_max:
                break
            if direction == -1:
                ret = crop
                break
            T -= 10
            direction = 1
        elif perc < tconf.white_min:
            if T < tconf.threshold_min:
                break
            if direction == 1:
                ret = crop
                break
            T += 10
            direction = -1
        else:
            ret = crop
            break
    return ret

# Calculate shift and turn based on vector
def check_shift_turn(angle, shift):
    turn_state = 0
    if angle < tconf.turn_angle or angle > 180 - tconf.turn_angle:
        turn_state = np.sign(90 - angle)
    shift_state = 0
    if abs(shift) > tconf.shift_max:
        shift_state = np.sign(shift)
    return turn_state, shift_state

def get_turn(turn_state, shift_state):
    turn_dir = 0
    turn_val = 0
    if shift_state != 0:
        turn_dir = shift_state
        turn_val = tconf.shift_step if shift_state != turn_state else tconf.turn_step
    elif turn_state != 0:
        turn_dir = turn_state
        turn_val = tconf.turn_step
    return turn_dir, turn_val

# Main loop for decision-making
def process_image(image_path):
    image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)

    global img, img_cropped
    img = image
    img_cropped = Roi.crop_roi(image)

    if image is None:
        print("Error: Image could not be read.")
        return

    balanced_image = balance_pic(image)

    if balanced_image is None:
        print("Error: Unable to process image.")
        return

    while True:
        angle, shift = get_vector(balanced_image)

        if angle is None:
            print("Error: Unable to find the line.")
            break

        turn_state, shift_state = check_shift_turn(angle, shift)
        turn_dir, turn_val = get_turn(turn_state, shift_state)

        if turn_dir != 0:
            direction = "Left" if turn_dir == -1 else "Right"
            print(f"Turn {direction}, Angle: {turn_val}")
        else:
            print("Go Straight")
            break

# Helper ROI and vector functions
class Roi:
    @staticmethod
    def crop_roi(image):
        h, w = image.shape
        return image[h // 2 :, :]

    @staticmethod
    def get_area():
        return 640 * 480 // 2

def get_vector(image):
    contours, _ = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None

    largest_contour = max(contours, key=cv.contourArea)
    moments = cv.moments(largest_contour)
    if moments['m00'] == 0:
        return None, None

    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])

    h, w = image.shape
    shift = (w // 2) - cx

    rows, cols = image.shape[:2]
    [vx, vy, x, y] = cv.fitLine(largest_contour, cv.DIST_L2, 0, 0.01, 0.01)
    angle = np.arctan2(-vy, vx) * 180 / np.pi
    if angle < 0:
        angle += 180

    return angle, shift

# Main camera loop
def main():
    picam2 = Picamera2()
    picam2.start()

    try:
        while True:
            image_path = "test.jpg"
            picam2.capture_file(image_path)
            print("Image captured and saved as 'test.jpg'")
            
            process_image(image_path)
            
            time.sleep(0.5)  # Delay between captures
    except KeyboardInterrupt:
        print("Stopping the image capture loop.")
    finally:
        picam2.stop()

if __name__ == "__main__":
    main()
