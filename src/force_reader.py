import serial.tools.list_ports
import serial
import math

SENSOR_VID_PID = "VID:PID=0483:5740"
ARDUINO_VID_PID = "VID:PID=2341:0043"
no_arduino = False


def find_port(VID_PID):
    for port in serial.tools.list_ports.comports():
        if VID_PID in port.hwid:
            return port.device

    return 0


def get_force_val(sensor):
    sensor.reset_input_buffer()
    sensor.write(b'?')  # requests value
    bytes_to_read = sensor.in_waiting
    sensor_val = sensor.read(bytes_to_read).decode('utf-8')

    if sensor_val != "":
        return sensor_val


def main():
    try:
        arduino = serial.Serial(
            find_port(ARDUINO_VID_PID), 115200, write_timeout=0, timeout=1)
        no_arduino = False
    except (ValueError):
        print("ARDUINO IS NOT CONNECTED!")
        no_arduino = True

    force_sensor = serial.Serial(find_port(SENSOR_VID_PID), 115200)

    print("Enter number of iterations: ")
    ITERATIONS = input().encode()
    print(ITERATIONS)
    print("Enter wanted force: ")
    WANTED_FORCE = input().encode()

    arduino.write(ITERATIONS)
    arduino.write(WANTED_FORCE)

    while True:
        sensor_val = get_force_val(force_sensor)

        if sensor_val is not None:
            sensor_val = sensor_val[:sensor_val.find('.') + 2]
            # print(sensor_val)
            if not no_arduino:
                arduino.write(str(-1 * abs(float(sensor_val))).encode())
                answer = arduino.read(arduino.in_waiting)
                if answer != b'':
                    print("got: ", answer)

                if answer == b"fin":
                    print("FINNISHED ALL ITERATIONS!")
                    break


main()
