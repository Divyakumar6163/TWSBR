import cv2
import numpy as np

# Create a blank white image
image = np.ones((768, 768, 3), dtype=np.uint8) * 255

# Draw a black line
start_point = (0, 0)  # Starting point of the line
end_point = (1200, 900)    # Ending point of the line
color = (0, 0, 0)         # Black color
thickness = 35             # Thickness of the line

# Draw the line on the image
cv2.line(image, start_point, end_point, color, thickness)

# Save or display the image
cv2.imwrite("black_tape.jpg", image)
# cv2.imshow("Black Tape", image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
