import serial
import time


def setup_serial():
    ser = serial.Serial('/dev/serial0', 9600, timeout=1)
    # serial is already open
    return ser


def write_direction(ser, direction):
    try:
        ser.write(direction)
        time.sleep(0.05)

    except KeyboardInterrupt:
        end_serial()


def end_serial(ser):
    print "Closing serial ports."
    ser.close()
