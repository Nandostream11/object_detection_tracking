import cv2
import numpy as np
import matplotlib.pyplot as plt

def detect_shapes(image_path):
    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Could not load image.")
        return

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply threshold
    # _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY_INV)
    # Alternatively, use adaptive thresholding for varying lighting conditions
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define green range (tune if needed)
    lower_green = np.array([35, 0, 0])   # include yellow-green shades
    upper_green = np.array([95, 255, 255]) # include teal/cyan
    
    for contour in contours:
        # Calculate perimeter
        peri = cv2.arcLength(contour, True)

        # Approximate the contour
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

        # Get bounding rect for aspect ratio check
        x, y, w, h = cv2.boundingRect(approx)

        # Classify shape
        shape = "None"
        vertices = len(approx)

        if vertices == 3:
            shape = "Triangle"
        elif vertices == 4:
            # Use minAreaRect to handle rotated squares
            rect = cv2.minAreaRect(approx)
            (cx, cy), (w, h), angle = rect
            if h == 0 or w == 0:
                continue
            aspect_ratio = min(w, h) / max(w, h)
            if 0.95 <= aspect_ratio <= 1.05:
                shape = "Square"
            else:
                shape = "Rectangle"
#        elif vertices == 5:
#           shape = "Pentagon"
#        elif vertices == 6:
#            shape = "Hexagon"
#        elif vertices > 6:
#            area = cv2.contourArea(contour)
#            if area > 100:
#                shape = "Circle"
        
        if shape in ["Triangle", "Square"]:
            # Create mask for this contour
            mask = np.zeros(hsv.shape[:2], dtype="uint8")
            cv2.drawContours(mask, [contour], -1, 255, -1)

            # Calculate percentage of green pixels inside contour
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            green_pixels = cv2.countNonZero(cv2.bitwise_and(green_mask, green_mask, mask=mask))
            total_pixels = cv2.countNonZero(mask)

            if total_pixels > 0 and (green_pixels / total_pixels) > 0.5:  # at least 50% green
                # Draw only green triangles/squares
                cv2.drawContours(image, [approx], -1, (225, 0, 0), 2)
                cv2.putText(image, f"{shape}", (x-5, y + 34),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (225, 0, 0), 2)

    # Convert BGR → RGB for matplotlib
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Show results in Spyder
#   plt.subplot(1, 2, 1)
    plt.imshow(image_rgb)
    plt.title("Detected Shapes")
    plt.axis("off")

#    plt.subplot(1, 2, 2)
#    plt.title("Threshold")
#    plt.imshow(thresh, cmap="gray")
#    plt.axis("off")

    plt.show()

def main(args=None):
    detect_shapes("resource/color-shapes.jpg")  # image path

if __name__ == "__main__":
    main()