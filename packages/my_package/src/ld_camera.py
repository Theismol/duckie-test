import cv2
import numpy as np

# Open a connection to the camera (0 represents the default camera, but it may vary)
cap = cv2.VideoCapture(0)

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Get the height and width of the frame
    height, width = frame.shape[:2]

    # Apply Gaussian blur to reduce noise

    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for yellow color in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])

    # Define the lower and upper bounds for white color in BGR
    lower_white = np.array([200, 200, 200])
    upper_white = np.array([255, 255, 255])

    # Threshold the image to get yellow and white regions
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    white_mask = cv2.inRange(frame, lower_white, upper_white)

    # Combine the masks to get the regions of interest
    combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

    # Crop the bottom half of the mask
    combined_mask_bottom_half = combined_mask[height // 2:, :]

    # Apply edge detection on the cropped mask
    edges = cv2.Canny(combined_mask_bottom_half, 100, 200)

    # Apply Hough Transform to detect lines
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=30)

    # Create a copy of the original frame to draw lines on
    line_image = np.copy(frame)

    # Select the leftmost and rightmost lines with the lowest y-coordinates
    left_line = None
    right_line = None

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Draw the line on the image
            cv2.line(line_image, (x1, y1 + height // 2), (x2, y2 + height // 2), (0, 255, 0), 3)  # Green for yellow lines

            # Check if the line is on the left side and update left_line
            if x1 < width // 2 and x2 < width // 2 and (left_line is None or y1 < left_line[0][3]):
                left_line = line

            # Check if the line is on the right side and update right_line
            elif x1 > width // 2 and x2 > width // 2 and (right_line is None or y1 < right_line[0][3]):
                right_line = line

    # Draw the left and right lines with the lowest y-coordinates in blue and red, respectively
    if left_line is not None:
        x1, y1, x2, y2 = left_line[0]
        cv2.line(line_image, (x1, y1 + height // 2), (x2, y2 + height // 2), (255, 0, 0), 3)  # Blue for left line

    if right_line is not None:
        x1, y1, x2, y2 = right_line[0]
        cv2.line(line_image, (x1, y1 + height // 2), (x2, y2 + height // 2), (0, 0, 255), 3)  # Red for right line

    # Calculate the midpoint of the two lines
    if left_line is not None and right_line is not None:
        left_midpoint = ((left_line[0][0] + left_line[0][2]) // 2, (left_line[0][1] + left_line[0][3]) // 2 + height // 2)
        right_midpoint = ((right_line[0][0] + right_line[0][2]) // 2, (right_line[0][1] + right_line[0][3]) // 2 + height // 2)

        # Calculate the average midpoint
        midpoint = ((left_midpoint[0] + right_midpoint[0]) // 2, (left_midpoint[1] + right_midpoint[1]) // 2)

        # Draw a big black dot (circle) at the midpoint
        cv2.circle(line_image, midpoint, 10, (0, 0, 0), -1)

    # Display the image with the detected lines and midpoint
    cv2.imshow('Lane Detection', line_image)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the OpenCV window
cap.release()
cv2.destroyAllWindows()
