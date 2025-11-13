import cv2
import time

# Transform image into a binary format for reading
image = cv2.imread("sunflowers.jpeg")

if image is None:
    print("Image not found")
    exit()

# 0 is for vertical flips
image_flipped = cv2.flip(image, 0)

# Blur the image (area of 36 pixels surrounding each one)
image_flipped_blurred = cv2.blur(image_flipped, (6, 6))

# Display original image
cv2.imshow("Original image", image)

# Display Flipped image
cv2.imshow("Flipped image", image_flipped)

# Display the  transformed image
cv2.imshow("Blurred and Flipped image", image_flipped_blurred)

# Wait for any key to be pressed to continue
# Using 5 seconds since there's a bug that if one of the images is pressed
# the waitKey(0) doesnt work correctly
cv2.waitKey(5000)

# Close the window
cv2.destroyAllWindows()