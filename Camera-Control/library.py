import cv2

# set screen visual appearance
COLOR = (0, 0, 255)
THICKNESS = 2


def apply_masks(frame, greenUpperBound, greenLowerBound):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # creates a mask for the color, do dilatations and erosions
    # to remove small blobs in the mask
    mask = cv2.inRange(hsv, greenLowerBound, greenUpperBound)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask


def find_circle_contour(contours):
    # find the largest contour in the mask, then use it to compute
    # the minimunm enclosing circle and centroid
    largest_contour = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
    return (x, y), radius, largest_contour


def calculate_centroid(largest_contour):
    M = cv2.moments(largest_contour)
    if M["m00"] > 0:
        centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        return centroid


def draw_circle(frame, center, radius):
    cv2.circle(frame, (int(center[0]), int(center[1])), int(radius), COLOR, THICKNESS)


def find_screen_position(centroid, frame):
    if centroid[0] < (frame.shape[1] / 3):
        position = 'l'
    elif centroid[0] < ((frame.shape[1] / 3) * 2):
        position = 'f'
    else:
        position = 'r'

    # if centroid[1] < (frame.shape[0]/3):
    #     position += "-Cima"
    # elif centroid[1] < ((frame.shape[0] / 3) * 2):
    #     position += "-Centro"
    # else:
    #     position += "-Baixo"

    return position


def print_distance(frame, distance):
    cv2.putText(frame, "%.2f cm" % distance, (frame.shape[1] - 200, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, COLOR, THICKNESS)


def print_position(frame, position):
    cv2.putText(frame, position, (50, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, COLOR, THICKNESS)
