import cv2 as cv
import numpy as np

# Configuration settings
class tconf:
    white_min = 10    # Minimum percentage of black pixels (tape)
    white_max = 70   # Maximum percentage of black pixels
    threshold_min = 50
    threshold_max = 200
    th_iterations = 10
    turn_angle = 20  # Angle threshold for turning
    shift_max = 30   # Maximum acceptable shift from center
    shift_step = 5   # Degree of turn for large shifts
    turn_step = 10   # Degree of turn for angle-based turns
    straight_run = 0.1  # Delay for straight movement (in seconds)

# Adjust the threshold dynamically
def balance_pic(image):
    global T
    T = 100  # Initial threshold
    ret = None
    direction = 0
    for i in range(tconf.th_iterations):
        _, gray = cv.threshold(image, T, 255, cv.THRESH_BINARY_INV)  # Look for black regions
        crop = Roi.crop_roi(gray)
        nwh = cv.countNonZero(crop)
        perc = int(100 * nwh / Roi.get_area())
        
        print('T', T)

        # Save the thresholded image and cropped image
        cv.imwrite(f"thresholded_image_{i}.png", gray)
        cv.imwrite(f"cropped_image_{i}.png", crop)

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

def get_turn(turn_state, shift_state,angle):
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
    # Read and preprocess the image
    image = cv.imread(image_path, cv.IMREAD_GRAYSCALE)
    
    global img,img_cropped
    img=image
    img_cropped = Roi.crop_roi(image)

    if image is None:
        print("Error: Image could not be read.")
        return
    
    balanced_image = balance_pic(image)
    
    # cv.imshow('BP',balanced_image);
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    
    if balanced_image is None:
        print("Error: Unable to process image.")
        return

    while True:
        a, shift = get_vector(balanced_image)

        print("Actual Angle",a)

        if a is None:
            print("Error: Unable to find the line.")
            break

        turn_state, shift_state = check_shift_turn(a, shift)
        turn_dir, turn_val = get_turn(turn_state, shift_state,a)

        if turn_dir != 0:
            # print(turn_dir)
            direction = "Left" if turn_dir == -1 else "Right"
            print(f"Turn {direction}, Angle: {turn_val}")
        else:
            print("Go Straight")
            break
        break

# Helper ROI and vector functions
class Roi:
    @staticmethod
    def crop_roi(image):
        h, w = image.shape
        return image[h // 2 :, :]  # Focus on the bottom half of the image

    @staticmethod
    def get_area():
        return 640 * 480 // 2  # Example size (modify based on actual resolution)

def get_vector(image):
    # Find contours to determine the tape position
    contours, _ = cv.findContours(image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Draw all contours and save the image
    cv.drawContours(img_cropped, contours, -1, (255, 255, 255), 2)
    cv.imwrite("contours.png", img_cropped)

    if not contours:
        return None, None

    # Find the largest contour and draw it
    largest_contour = max(contours, key=cv.contourArea)
    cv.drawContours(img_cropped, [largest_contour], -1, (255, 255, 255), 2)
    cv.imwrite("largest_contour.png", img_cropped)

    moments = cv.moments(largest_contour)
    if moments['m00'] == 0:
        return None, None

    # Calculate the centroid of the largest contour
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    print("Cy and Cx:", cy, cx)

    h, w = image.shape
    shift = (w // 2) - cx

    # Mark the centroid on the image
    cv.circle(img_cropped, (cx, cy), 5, (255, 255, 255), -1)
    cv.imwrite("centroid_marked.png", img_cropped)

    # Fit a line to the largest contour
    rows, cols = image.shape[:2]
    [vx, vy, x, y] = cv.fitLine(largest_contour, cv.DIST_L2, 0, 0.01, 0.01)
    angle = np.arctan2(-vy, vx) * 180 / np.pi
    if angle < 0:
        angle += 180
    print("Angle:", angle)
    
    return angle, shift

# Example usage
process_image(r"C:\Users\AYUSHI JAIN\OneDrive\Desktop\PathFollowingTesting\black_tape.jpg")