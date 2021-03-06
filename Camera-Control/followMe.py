# imports
import numpy as np
import cv2
import imutils
import library
import serial
import time
import threading
import ringbuffer as rb

SER = serial.Serial('/dev/serial0', 19200, timeout=1) #19200
LOCK = threading.Lock() # Mutex for writing in serial port

# HSV threshold for green color
#GREEN_LOWER = [0, 137, 90]
#GREEN_UPPER = [7, 188, 240]

# HSL threshold for green color
# GREEN_LOWER = [1, 88, 0]
# GREEN_UPPER = [11, 151, 255]

# Luv threshold for green color
GREEN_LOWER = [0, 137, 0]
GREEN_UPPER = [255, 170, 255]

# Constants used to calibrate the camera, in cm
INITIAL_DISTANCE = 34.5 #16 #31
OBJECT_WIDTH = 5.5 #3.75 # radius size
OBJECT_APPARENT_WIDTH = 44 #114 #39 # radius in pixels
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
LEFT = 'r'
RIGHT = 'l'
TURN_LEFT = 'e'
TURN_RIGHT = 'd'
STOP = 's'
NONE = 'n'

# Time constants
LONGER_WAIT = 0.05
LONG_WAIT = 0.0125
MEDIUM_WAIT = 0.00625
SHORT_WAIT = 0.00625

# Distance constants in cm
MIN_DISTANCE = 20
MAX_DISTANCE = 50

BUFFER_SIZE = 10 # At 30 fps this is 0.5 sec equivalent

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
    current_frame = imutils.resize(current_frame, width=400)
    # isolate the color in a binary image
    mask = library.apply_masks(current_frame, COLOR_UPPER_BOUND, COLOR_LOWER_BOUND)
    # find contours in the mask and initialize the current (x,y) center
    contours_found = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cv2.imshow("Mask Luv", mask)
    return contours_found, current_frame


def setup_initial_vars():
    initial_position = NONE
    initial_radius = 1
    initial_distance = calculate_distance(initial_radius)

    return initial_position, initial_distance, initial_radius


def end_serial():
    print "Closing serial ports"
    SER.close()


def go(direction, sleep):
    print "go " + direction
    try:
        with LOCK:
            SER.write(direction)
        time.sleep(sleep)
    except KeyboardInterrupt:
        end_serial()


def stop():
    go(STOP, LONGER_WAIT)


def turn_left(sleep):
    go(TURN_LEFT, sleep)
    stop()
    

def turn_right(sleep):
    go(TURN_RIGHT, sleep)
    stop()


def is_moving():
    global distance
    global last_distance
    z_axys = ((last_distance - distance) > 0.25)

    global position
    global last_position
    x_axys = (last_position != position)

    return z_axys and x_axys


position, distance, current_radius = setup_initial_vars()
last_position = position
last_distance = distance

distances = rb.RingBuffer(BUFFER_SIZE)

# if a video path was not supplied, grab the webcam
# otherwise, grab the reference video
camera = cv2.VideoCapture(0)
if camera.isOpened():
    (grab, frame) = camera.read()
    cv2.imshow("Frame", frame)
    time.sleep(0.001) # Time to warm up the camera

while camera.isOpened():

    contours, frame = get_contours_in_frame()
    
    # only proceed if at least one contour was found
    if len(contours) > 0:
        ((x, y), current_radius, largest_contour) = library.find_circle_contour(contours)
        
        distance, position = get_distance_n_position(current_radius, (x, y), frame, largest_contour)

        distances.append(distance)
        print ('Media:')
        print rb.running_mean(distances.get(), BUFFER_SIZE)
        # The cart is inside the safe distance
        # Keep inside it
        if MIN_DISTANCE - 1 < distance < MIN_DISTANCE + 1: # Error margin
            wait_time = SHORT_WAIT
            position = STOP

        elif MIN_DISTANCE < distance < MAX_DISTANCE:
            wait_time = SHORT_WAIT

        # Cart is too far way, accelerate
        elif distance > MAX_DISTANCE:
            wait_time = LONG_WAIT

        # Cart is inside the danger zone. back up
        elif distance < MIN_DISTANCE:
            wait_time = SHORT_WAIT
            position = BACKWARD

        # Cart is in the limbo, wait for rescue
        else:
            wait_time = SHORT_WAIT
            position = NONE

        go(position, wait_time)

        last_position = position
        last_distance = distance
                
    else:
        if last_position == LEFT:
            distance = 0
            turn_left(LONGER_WAIT)

        else:
            distance = 0
            turn_right(LONGER_WAIT)

    print distance
    print '\n'
    # print "FPS {0}".format(camera.get(cv2.CAP_PROP_FPS))
    
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
    
# cleanup the camera and close any connections
end_serial()
camera.release()
cv2.destroyAllWindows()
