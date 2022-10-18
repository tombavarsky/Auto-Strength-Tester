import serial.tools.list_ports


ports = serial.tools.list_ports.comports()

for port, desc, hwid in sorted(serial.tools.list_ports.comports()):
    print("{}: {} [{}]".format(port, desc, hwid))


def find_arduino_port():
    for port in serial.tools.list_ports.comports():
        if 'VID:PID=0483:5740' in port.hwid:
            return port.device
        return 0


def get_goal_char(connection):
    # connection.reset_input_buffer()
    connection.write(b'?')
    bytes_to_read = connection.in_waiting
    goal_char = connection.read(bytes_to_read).decode('UTF-8')
    return goal_char


def main():
    connection = serial.Serial(
        find_arduino_port(), 115200, timeout=1, parity=serial.PARITY_SPACE)
    while True:
        print(get_goal_char(connection))


main()
