import serial.tools.list_ports
import serial
import csv
import time
import tkinter as tk
from tkinter import ttk

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


def b_click():
    ...


def popup():
    root = tk.Tk()

    iterations_tk = tk.IntVar()
    wanted_force_tk = tk.DoubleVar()

    root.geometry("300x150")
    root.title('Auto Strength Tester')
    main_frame = ttk.Frame(root)
    main_frame.pack(padx=10, pady=10, fill='x', expand=True)

    iterations_label = ttk.Label(main_frame, text="Iterations:")
    iterations_label.pack(fill='x', expand=True)

    iterations_entry = ttk.Entry(main_frame, textvariable=iterations_tk)
    iterations_entry.pack(fill='x', expand=True)
    iterations_entry.focus()

    wanted_force_label = ttk.Label(main_frame, text="Force:")
    wanted_force_label.pack(fill='x', expand=True)

    wanted_force_entry = ttk.Entry(main_frame, textvariable=wanted_force_tk)
    wanted_force_entry.pack(fill='x', expand=True)

    button = ttk.Button(main_frame, text="start",
                        command=lambda: root.destroy())
    button.pack(fill='x', expand=True, pady=10)

    root.mainloop()

    return (str(iterations_tk.get()).encode(), str(wanted_force_tk.get()).encode())


def main():
    try:
        arduino = serial.Serial(
            find_port(ARDUINO_VID_PID), 115200, write_timeout=0, timeout=1)
        no_arduino = False
    except (ValueError):
        print("ARDUINO IS NOT CONNECTED!")
        no_arduino = True

    force_sensor = serial.Serial(find_port(SENSOR_VID_PID), 115200)

    ITERATIONS, WANTED_FORCE = popup()

    # print("Enter number of iterations: ")
    # ITERATIONS = input().encode()
    # print("Enter wanted force: ")
    # WANTED_FORCE = input().encode()

    arduino.reset_input_buffer()
    arduino.reset_output_buffer()

    print("sent: ", ITERATIONS, " - ", WANTED_FORCE)

    arduino.write(b'<')
    arduino.write(ITERATIONS)
    arduino.write(b'>')
    arduino.write(b'\n')
    arduino.write(b'<')
    arduino.write(WANTED_FORCE)
    arduino.write(b'>')

    # answer = b""
    ITERATION_SIGN = b"it"
    FINNISH_SIGN = b"fin"
    START_TIME = time.time()
    WRITE_RESULUTION = 0.00
    write_time = 0

    curr_iteration = 0
    curr_time = 0.0
    sensor_val = 0.0

    header = ["Iteration", "Time", "Force"]
    data = [curr_iteration, curr_time, sensor_val]
    f = open("src/results.txt", 'w')
    writer = csv.writer(f)

    writer.writerow(header)

    while True:
        curr_time = round(time.time() - START_TIME, 2)
        sensor_val = get_force_val(force_sensor)

        if sensor_val is not None:
            sensor_val = sensor_val[:sensor_val.find('N')]
            # print(sensor_val)
            if not no_arduino:
                # arduino.write('<'.encode())
                # arduino.write(str(abs(float(sensor_val))).encode())
                arduino.write(sensor_val.encode())
                # print(str(-1 * abs(float(sensor_val))))
                # arduino.write('>'.encode())
                answer = arduino.read(arduino.in_waiting)
                # while True:
                #     answer += arduino.read()

                #     if b"\n" in answer:
                #         print(answer)
                #         break

                if answer != b'':
                    print("got", answer)

                if ITERATION_SIGN in answer:
                    curr_iteration += 1
                    print("NEXT ITERATION!! ", curr_iteration)

                data = [curr_iteration, curr_time, sensor_val]
                if curr_time - write_time >= WRITE_RESULUTION:
                    write_time = curr_time
                    writer.writerow(data)

                if FINNISH_SIGN in answer:
                    print("FINNISHED ALL ITERATIONS!")
                    f.close()

                    break


main()
