# imports
import numpy as np
import imutils
import cv2
import library
import comArduino as com
import time

# HSV threshold for green color
GREEN_LOWER = [48, 48, 49]
GREEN_UPPER = [73, 255, 220]

# Constants used to calibrate the camera, in cm
INITIAL_DISTANCE = 16
OBJECT_WIDTH = 5.5  # radius size
OBJECT_APPARENT_WIDTH = 114  # radius in pixels
FOCAL_LENGTH = (OBJECT_APPARENT_WIDTH * INITIAL_DISTANCE) / OBJECT_WIDTH

# Filter noise that are less than 10 pixels in radius
MINIMUM_APPARENT_WIDTH = 10

# Define the lower and upper bounds of the color
# Use 8 bits unsigned numbers (0 - 255)
COLOR_LOWER_BOUND = np.array(GREEN_LOWER, np.uint8)
COLOR_UPPER_BOUND = np.array(GREEN_UPPER, np.uint8)

# Position constants
FORWARD = 'f'
BACKWARD = 'b'
LEFT = 'l'
RIGHT = 'r'
TURN_LEFT = 'e'
TURN_RIGHT = 'd'
STOP = 's'
NONE = 'n'

# Time constants
LONGER_WAIT = 0.5
LONG_WAIT = 0.05
MEDIUM_WAIT = 0.0125
SHORT_WAIT = 0.00625

# Distance constants in cm
MIN_DISTANCE = 20
MAX_DISTANCE = 50


def calculate_distance(radius):
    current_distance = (OBJECT_WIDTH * FOCAL_LENGTH) / radius
    return current_distance


def get_distance_n_position(radius, (center_x, center_y),
                            current_frame, largest_con):
    """

    :param radius: radius of the object identified
    :type radius: double
    :param current_frame: OpenCV matrix of a still image
    :param largest_con: Contour of the object identified
    :return: Distance calculated and position caught
    :rtype: (double, char)
    """
    # This is set to prevent using caught noise as a object
    if radius > 10:
        # draws the circle and centroid on the frame then update the list of
        # tracked points
        library.draw_circle(current_frame,
                            (int(center_x), int(center_y)),
                            int(radius))

        known_distance = calculate_distance(current_radius)

        centroid = library.calculate_centroid(largest_con)
        known_position = library.find_screen_position(centroid, current_frame)

    else:
        known_distance = 0.0
        known_position = NONE

    return known_distance, known_position


def get_contours_in_frame():
    # grab current frame
    (grabbed, current_frame) = camera.read()
    # resize frame to save processing time
    current_frame = imutils.resize(current_frame, width=600)
    # isolate the color in a binary image
    mask = library.apply_masks(current_frame, COLOR_UPPER_BOUND, COLOR_LOWER_BOUND)
    # find contours in the mask and initialize the current (x,y) center
    contours_found = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    return contours_found, current_frame


def setup_initial_vars():
    initial_position = NONE
    initial_radius = 1
    initial_distance = calculate_distance(initial_radius)

    return initial_position, initial_distance, initial_radius


def go(direction, sleep, serial_com):
    com.write_direction(serial_com, direction)
    time.sleep(sleep)


def stop(serial_com):
    com.write_direction(serial_com, STOP)
    time.sleep(SHORT_WAIT)


def turn_left(sleep, serial_com):
    com.write_direction(serial_com, TURN_LEFT)
    time.sleep(sleep)


def turn_right(sleep, serial_com):
    com.write_direction(serial_com, TURN_RIGHT)
    time.sleep(sleep)


position, distance, current_radius = setup_initial_vars()
last_position = position
ser = com.setup_serial()

# if a video path was not supplied, grab the webcam
# otherwise, grab the reference video
camera = cv2.VideoCapture(0)

while camera.isOpened():

    contours, frame = get_contours_in_frame()

    # only proceed if at least one contour was found
    if len(contours) > 0:
        ((x, y), current_radius, largest_contour) = library.find_circle_contour(contours)

        distance, position = get_distance_n_position(current_radius, (x, y), frame, largest_contour)

        if MIN_DISTANCE < distance < MAX_DISTANCE:
            wait_time = MEDIUM_WAIT

        elif distance > MAX_DISTANCE:
            wait_time = LONG_WAIT

        elif distance < MIN_DISTANCE:
            wait_time = SHORT_WAIT

        else:
            wait_time = SHORT_WAIT

        go(position, wait_time, ser)

    elif last_position == LEFT or last_position == NONE:
        turn_left(SHORT_WAIT, ser)

    elif last_position == RIGHT:
        turn_right(SHORT_WAIT, ser)

    last_position = position

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup the camera and close any connections
com.end_serial(ser)
camera.release()
cv2.destroyAllWindows()
