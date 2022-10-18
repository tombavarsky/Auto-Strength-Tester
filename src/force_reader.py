import serial.tools.list_ports
import serial

SENSOR_VID_PID = "VID:PID=0483:5740"
ARDUINO_VID_PID = "VID:PID=2341:0043"


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
    arduino = serial.Serial(find_port(ARDUINO_VID_PID), 115200)
    force_sensor = serial.Serial(find_port(SENSOR_VID_PID), 115200)

    while True:
        sensor_val = get_force_val(force_sensor)

        if sensor_val is not None:
            sensor_val = float(sensor_val[:4])
            print(sensor_val)

            arduino.write(sensor_val)


main()
