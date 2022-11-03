import numpy as np
import serial.tools.list_ports
import serial
import csv
import time
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt


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


def popup():
    root = tk.Tk()

    iterations_tk = tk.IntVar()
    wanted_force_tk = tk.DoubleVar()

    root.geometry("300x200")
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

    options = ('push', 'pull')

    options_var = tk.Variable(value=options)

    listbox = tk.Listbox(
        main_frame,
        listvariable=options_var,
        height=2
    )

    listbox.pack(expand=True, fill=tk.BOTH)

    selected_item = ""
    started = False

    def get_item(event):
        nonlocal selected_item

        selected_item = str(listbox.get(listbox.curselection()))

    def b_click():
        nonlocal started
        started = True
        root.destroy()

    listbox.bind('<<ListboxSelect>>', get_item)

    button = ttk.Button(main_frame, text="start",
                        command=b_click)
    button.pack(fill='x', expand=True, pady=10)

    root.mainloop()

    return (str(iterations_tk.get()).encode(), str(wanted_force_tk.get()).encode(), selected_item.encode(), started)


def main():
    SENSOR_VID_PID = "VID:PID=0483:5740"
    ARDUINO_VID_PID = "VID:PID=2341:0043"
    ITERATION_SIGN = b"i"
    FINNISH_SIGN = b"f"
    WRITE_RESOLUTION = 0.00

    write_time = 0
    no_arduino = False

    curr_iteration = 0
    curr_time = 0.0
    sensor_val = 0.0
    last_sensor_val = 0.0
    last_last_sensor_val = 0.0

    header = ["Iteration", "Time", "Force"]
    data = [curr_iteration, curr_time, sensor_val]
    f = open("C:/Users/user/Documents/PlatformIO/Projects/Auto-Strength-Tester/src/results.txt", 'w', newline='')
    writer = csv.writer(f)

    writer.writerow(header)

    try:
        arduino = serial.Serial(
            find_port(ARDUINO_VID_PID), 115200, write_timeout=0, timeout=1)
        no_arduino = False
    except (ValueError):
        print("ARDUINO IS NOT CONNECTED!")
        no_arduino = True

    force_sensor = serial.Serial(find_port(SENSOR_VID_PID), 115200)

    ITERATIONS, WANTED_FORCE, PUSH, STARTED = popup()

    if ITERATIONS == b'0' or WANTED_FORCE == b'0.0' or not STARTED:
        return

    print("push: ", PUSH)

    arduino.reset_input_buffer()
    arduino.reset_output_buffer()

    PUSH_SEND = b'1' if PUSH == b'push' else b'2'

    print("sent: ", ITERATIONS, " - ", WANTED_FORCE,
          " - ", PUSH_SEND)

    # sending data to arduino
    arduino.write(b'<')
    arduino.write(ITERATIONS)
    arduino.write(b'>')
    arduino.write(b'\n')

    arduino.write(b'<')
    arduino.write(WANTED_FORCE)
    arduino.write(b'>')
    arduino.write(b'\n')

    arduino.write(b'<')
    arduino.write(PUSH_SEND)
    arduino.write(b'>')

    START_TIME = time.time()
    curr_time_list = []
    force_val_list = []

    while True:
        curr_time = round(time.time() - START_TIME, 4)
        sensor_val = get_force_val(force_sensor)

        if sensor_val is not None:
            sensor_val = sensor_val[:sensor_val.find('N')]
            print(sensor_val)
            if (float(last_sensor_val) == 0 and float(sensor_val) != 0) or (float(last_last_sensor_val) == 0 and float(last_sensor_val) != 0 and float(sensor_val) != 0):
                last_sensor_val = sensor_val
                last_last_sensor_val = last_sensor_val
                sensor_val = 0

                continue

            if not no_arduino:
                # arduino.write('<'.encode())
                arduino.write(sensor_val.encode())
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

                if curr_time - write_time >= WRITE_RESOLUTION:
                    data = [curr_iteration, curr_time, sensor_val]
                    curr_time_list.append(curr_time)
                    force_val_list.append(abs(float(sensor_val)))
                    write_time = curr_time
                    writer.writerow(data)

                if FINNISH_SIGN in answer:
                    print("FINNISHED ALL ITERATIONS!")
                    f.close()
                    arduino.close()
                    force_sensor.close()

                    fig, ax = plt.subplots()
                    plt.xticks(np.arange(0, max(curr_time_list) + 1, 1))
                    plt.yticks(np.arange(0, max(force_val_list) + 1, 0.5))
                    plt.setp(ax.get_xticklabels(), fontsize=7)
                    plt.setp(ax.get_yticklabels(), fontsize=6)

                    plt.plot(curr_time_list, force_val_list)
                    plt.grid(True)
                    plt.show()

                    break
            last_sensor_val = sensor_val
            last_last_sensor_val = last_sensor_val


main()
