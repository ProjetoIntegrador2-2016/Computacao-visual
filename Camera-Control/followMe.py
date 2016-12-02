# imports
import numpy as np
import cv2
import imutils
import library
import serial
import time
import threading
import ringbuffer as rb
import struct
import math

SER = serial.Serial('/dev/serial0', 9600, timeout=1)  # 19200

LOCK = threading.Lock()  # Mutex for writing in serial port

# HSV threshold for green color
# GREEN_LOWER = [0, 137, 90]
# GREEN_UPPER = [7, 188, 240]

# HSL threshold for green color
# GREEN_LOWER = [1, 88, 0]
# GREEN_UPPER = [11, 151, 255]

# Luv threshold for green color
GREEN_LOWER = [0, 126, 0]
GREEN_UPPER = [255, 255, 255]

# Constants used to calibrate the camera, in cm
INITIAL_DISTANCE = 34.5  #16 #31
OBJECT_WIDTH = 5.5  #3.75 # radius size
OBJECT_APPARENT_WIDTH = 44  #114 #39 # radius in pixels
FOCAL_LENGTH = (OBJECT_APPARENT_WIDTH * INITIAL_DISTANCE) / OBJECT_WIDTH

# Filter noise that are less than 10 pixels in radius
MINIMUM_APPARENT_WIDTH = 10

# Define the lower and upper bounds of the color
# Use 8 bits unsigned numbers (0 - 255)
COLOR_LOWER_BOUND = np.array(GREEN_LOWER, np.uint8)
COLOR_UPPER_BOUND = np.array(GREEN_UPPER, np.uint8)

BUFFER_SIZE = 10  # At 30 fps this is 0.5 sec equivalent


def calculate_distance(radius):
    current_distance = (OBJECT_WIDTH * FOCAL_LENGTH) / radius
    return current_distance


def calculate_deviation_cm(current_deviaton):
    deviation_cm = (current_deviaton * OBJECT_WIDTH) / OBJECT_APPARENT_WIDTH
    return deviation_cm


def get_distance_n_position(radius, (center_x, center_y),
                            current_frame, largest_con):
    """

    :param radius: radius of the object identified
    :type radius: double
    :param current_frame: OpenCV matrix of a still image
    :param largest_con: Contour of the object identified
    :return: Distance calculated and deviation from center line
    :rtype: (double, double)
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
        known_deviation = library.find_screen_position(centroid, current_frame)

    else:
        known_distance = 0.0
        known_deviation = 0.0

    return known_distance, known_deviation


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
    initial_deviation = 0.0
    initial_radius = 1
    initial_distance = calculate_distance(initial_radius)

    return initial_deviation, initial_distance, initial_radius


def write_to_serial(string):
    try:
        with LOCK:
            SER.write(string)
            time.sleep(0.05)
    except KeyboardInterrupt:
        end_serial()


def end_serial():
    print ("Closing serial ports")
    SER.close()


deviation, distance, current_radius = setup_initial_vars()
distances = rb.RingBuffer(BUFFER_SIZE)
last_valid_teta = 0
last_xPos = 0

# if a video path was not supplied, grab the webcam
# otherwise, grab the reference video
camera = cv2.VideoCapture(0)
if camera.isOpened():
    (grab, frame) = camera.read()
    #cv2.imshow("Frame", frame)
    time.sleep(0.001)  # Time to warm up the camera

while camera.isOpened():

    contours, frame = get_contours_in_frame()
    
    # only proceed if at least one contour was found
    if len(contours) > 0:
        ((x, y), current_radius, largest_contour) = library.find_circle_contour(contours)
        
        distance, deviation = get_distance_n_position(current_radius, (x, y), frame, largest_contour)

        distances.append(distance)

        distance_mean = rb.running_mean(distances.get(), BUFFER_SIZE)

        if distance_mean > 0:
            deviated_cm = calculate_deviation_cm(deviation)
            tetaRad = np.arcsin(deviated_cm / distance_mean)

            # For invalid arcsin values is simpler to use the last valid one
            if not math.isnan(tetaRad):
                last_valid_teta = tetaRad
            else:
                tetaRad = last_valid_teta

            # Get in Y axys (straight ahead)
            yPos = distance_mean * (np.cos(tetaRad))
            xPos = deviated_cm

            # Put the space in front to assure the first float will be read
            msg = ("%.2f, %.2f" % (xPos, yPos))
            print msg
            write_to_serial(msg.encode('ascii'))

            last_xPos = xPos

        # else:
            # wait for mean

    else:
        invalid_token = ("%.2f, %.2f" % (last_xPos, -1.0))
        print invalid_token
        write_to_serial(invalid_token.encode('ascii'))

    # print "FPS {0}".format(camera.get(cv2.CAP_PROP_FPS))
    
    # show the frame to our screen
    #cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
    
# cleanup the camera and close any connections
end_serial()
camera.release()
cv2.destroyAllWindows()
