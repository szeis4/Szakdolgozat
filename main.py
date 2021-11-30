import serial
import math
from collections import deque
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import tkinter as tk

# adatszerkezet:
#   byte[0:1]   2C 4D   header
#   byte[2]     01      konstans
#   byte[3]     00      konstans
#   byte[4]     00v01   vezérlési mód (ST v PT)
#   byte[5]             amplitúdó (dinamika tartomány; y tengely felosztása; 255 -> -6Ncm-6Ncm)
#   byte[6]             skála/frekvencia (x tengely felosztása; 128 -> -180°-180°)
#   byte[7:366]         adat (a görbe pontjai)

# küldött függvények:
#   0: konstans (alapértelmezett)
#   1: lineáris
#   2: szinusz hiprebolikusz
#   3: köbös
#   4: szinusz hiperbolikusz + köbös

calculated_torques = []


def torque_calculator(function=0, tolerance=0, const_value=3):
    torques = []

    tolerance_in_rad = tolerance * math.pi / 180

    for i in range(-180, 180):
        i_in_rad = i * math.pi / 180
        t_min = -127
        t_max = 128

        if function == 0:
            torques.append(const_value)

        elif function == 1:
            r_min = -math.pi
            r_max = math.pi
            if -180 <= i < -tolerance:
                m = -(i_in_rad + tolerance_in_rad)
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif -tolerance <= i < tolerance:
                tor = round(((0 - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif tolerance <= i < 180:
                m = -(i_in_rad - tolerance_in_rad)
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)

        elif function == 2:
            r_min = math.sinh(-math.pi)
            r_max = math.sinh(math.pi)
            if -180 <= i < -tolerance:
                m = -(math.sinh(i_in_rad + tolerance_in_rad))
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif -tolerance <= i < tolerance:
                tor = round(((0 - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif tolerance <= i < 180:
                m = -(math.sinh(i_in_rad - tolerance_in_rad))
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)

        elif function == 3:
            r_min = pow(-math.pi, 3)
            r_max = pow(math.pi, 3)
            if -180 <= i < -tolerance:
                m = -(pow(i_in_rad + tolerance_in_rad, 3))
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif -tolerance <= i < tolerance:
                tor = round(((0 - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif tolerance <= i < 180:
                m = -(pow(i_in_rad - tolerance_in_rad, 3))
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)

        elif function == 4:
            r_min = (math.sinh(-math.pi) + pow(-math.pi, 3))
            r_max = (math.sinh(math.pi) + pow(math.pi, 3))
            if -180 <= i < -tolerance:
                m = -(math.sinh(i_in_rad + tolerance_in_rad) + pow(i_in_rad + tolerance_in_rad, 3))
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif -tolerance <= i < tolerance:
                tor = round(((0 - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)
            elif tolerance <= i < 180:
                m = -(math.sinh(i_in_rad - tolerance_in_rad) + pow(i_in_rad - tolerance_in_rad, 3))
                tor = round(((m - r_min) / (r_max - r_min)) * (t_max - t_min) + t_min) + 127
                torques.append(tor)

    if len(torques) == 0:
        print("Not known function!")
        print("Possible values:\n0: constant (default; default torque value: 3)\n1: linear\n2: hyperbolic "
              "sine\n3: cubic\n4: hyperbolic sine - cubic")

    return torques


def movement_data_create(trajectory, torques, amplitude=255, frequency=128):
    movement_data_arr = []

    values = torques

    rotatable_values = deque(values)

    for pos in trajectory:
        actual_step = [0x2c, 0x4d, 0x01, 0x00, 0x01, amplitude, frequency]
        rotatable_values.rotate(pos)
        for value in rotatable_values:
            actual_step.append(value)
        rotatable_values.rotate(-pos)
        movement_data_arr.append(actual_step)

    return movement_data_arr


def to_byte_data(original_data):
    byte_data = []
    for arr in original_data:
        byte_data.append(bytearray(arr))

    return byte_data


def send_to_port(port_name, send):
    try:
        ser = serial.Serial(port_name)
        ser.baudrate = 115200
        for data in send:
            ser.write(data)
        ser.close()
    except serial.serialutil.SerialException:
        popup_win = tk.Tk()
        popup_win.wm_title("Error")

        popup_msg = tk.Label(master=popup_win, text="COM3 port is not opened.")
        popup_msg.pack(side=tk.TOP)

        popup_btn = tk.Button(master=popup_win, text="OK", command=popup_win.destroy)
        popup_btn.pack(side=tk.BOTTOM)


def option_list_event(event):
    selected_fun = selected_function.get()
    t = np.arange(-np.pi, np.pi, np.pi / 180)
    torque_figure.clear()
    calc_torque = []
    if selected_fun == 'constant':
        calc_torque = torque_calculator(0, tolerance=int(tolerance_entry.get()), const_value=int(constant_entry.get()))
        torque_figure.add_subplot(111).plot(t, calc_torque)
        torque_canvas.draw()
    elif selected_fun == 'linear':
        calc_torque = torque_calculator(1, tolerance=int(tolerance_entry.get()))
        torque_figure.add_subplot(111).plot(t, calc_torque)
        torque_canvas.draw()
    elif selected_fun == "hyperbolic sine":
        calc_torque = torque_calculator(2, tolerance=int(tolerance_entry.get()))
        torque_figure.add_subplot(111).plot(t, calc_torque)
        torque_canvas.draw()
    elif selected_fun == "cubic":
        calc_torque = torque_calculator(3, tolerance=int(tolerance_entry.get()))
        torque_figure.add_subplot(111).plot(t, calc_torque)
        torque_canvas.draw()
    elif selected_fun == "hyperbolic sine + cubic":
        calc_torque = torque_calculator(4, tolerance=int(tolerance_entry.get()))
        torque_figure.add_subplot(111).plot(t, calc_torque)
        torque_canvas.draw()

    global calculated_torques
    calculated_torques = []
    calculated_torques = calc_torque


def create_trajectory(points):
    movement_trajectory = []
    stopping_points = points.split()
    for i in range(0, len(stopping_points)-1):
        if int(stopping_points[i]) > int(stopping_points[i+1]):
            step_dir = -1
        else:
            step_dir = 1
        for j in range(int(stopping_points[i]), int(stopping_points[i+1]), step_dir):
            movement_trajectory.append(j)
    return movement_trajectory


def button_send_action():

    if selected_function.get() == "Select a function":
        popup_win = tk.Tk()
        popup_win.wm_title("Error")

        popup_msg = tk.Label(master=popup_win, text="Select a trajectory function")
        popup_msg.pack(side=tk.TOP)

        popup_btn = tk.Button(master=popup_win, text="OK", command=popup_win.destroy)
        popup_btn.pack(side=tk.BOTTOM)

    else:
        trajectory = create_trajectory(trajectory_entry.get())

        movement_data = movement_data_create(trajectory, calculated_torques, int(amplitude_entry.get()), int(frequency_entry.get()))

        send_data = to_byte_data(movement_data)

        send_to_port("COM3", send_data)


if __name__ == '__main__':
    # movement_trajectory_positions = []
    # for i in range(91):
    #     movement_trajectory_positions.append(i)
    # for i in range(90, 45, -1):
    #     movement_trajectory_positions.append(i)
    #
    # test_trajectory_positions = []
    #
    # for i in range(360):
    #     test_trajectory_positions.append(i)
    #     test_trajectory_positions.append(i)
    #     test_trajectory_positions.append(i)
    #     test_trajectory_positions.append(i)
    #     test_trajectory_positions.append(i)
    #
    # move_data = movement_data_create(test_trajectory_positions)
    # send_data = to_byte_data(move_data)

    # create_gui()

    # send_to_port("COM3", send_data)

    window = tk.Tk()
    window.title("Motor driver")

    set_value_frame = tk.Frame(master=window)
    set_value_frame.grid(row=0, column=0, padx=5, pady=5)

    trajectory_frame = tk.Frame(master=set_value_frame)
    trajectory_frame.grid(row=0, column=0, padx=5, pady=5)
    trajectory_label = tk.Label(master=trajectory_frame,
                                text="Stopping points of the movement trajectory separated with spaces, including the starting point")
    trajectory_label.pack()
    trajectory_entry = tk.Entry(master=trajectory_frame)
    trajectory_entry.insert(tk.END, "0 360")
    trajectory_entry.pack(fill=tk.X)

    value_frame = tk.Frame(master=set_value_frame)
    value_frame.grid(row=0, column=1, padx=5, pady=5)

    tolerance_frame = tk.Frame(master=value_frame)
    tolerance_frame.grid(row=0, column=0, padx=5, pady=5)
    tolerance_label = tk.Label(master=tolerance_frame, text="Error tolerance in degree")
    tolerance_label.grid(row=1, column=0, padx=5, pady=5)
    tolerance_entry = tk.Entry(master=tolerance_frame)
    tolerance_entry.insert(tk.END, "0")
    tolerance_entry.grid(row=2, column=0, padx=5, pady=5)

    amplitude_frame = tk.Frame(master=value_frame)
    amplitude_frame.grid(row=3, column=0, padx=5, pady=5)
    amplitude_label = tk.Label(master=amplitude_frame, text="Amplitude value for the function")
    amplitude_label.grid(row=4, column=0, padx=5, pady=5)
    amplitude_entry = tk.Entry(master=amplitude_frame)
    amplitude_entry.insert(tk.END, "255")
    amplitude_entry.grid(row=5, column=0, padx=5, pady=5)

    frequency_frame = tk.Frame(master=value_frame)
    frequency_frame.grid(row=6, column=0, padx=5, pady=5)
    frequency_label = tk.Label(master=frequency_frame, text="Frequency value for the function")
    frequency_label.grid(row=7, column=0, padx=5, pady=5)
    frequency_entry = tk.Entry(master=frequency_frame)
    frequency_entry.insert(tk.END, "128")
    frequency_entry.grid(row=8, column=0, padx=5, pady=5)

    constant_frame = tk.Frame(master=value_frame)
    constant_frame.grid(row=9, column=0, padx=5, pady=5)
    constant_label = tk.Label(master=constant_frame, text="Value for the constant function")
    constant_label.grid(row=10, column=0, padx=5, pady=5)
    constant_entry = tk.Entry(master=constant_frame)
    constant_entry.insert(tk.END, "3")
    constant_entry.grid(row=11, column=0, padx=5, pady=5)

    value_frame.columnconfigure(0, weight=1)
    value_frame.rowconfigure([0, 11], weight=1)

    torque_function_frame = tk.Frame(master=window)
    torque_function_frame.grid(row=1, column=0, padx=5, pady=5)
    torque_function_label = tk.Label(master=torque_function_frame, text="Select the function for the torque correction")
    torque_function_label.pack(fill=tk.X)
    function_list = ['constant', 'linear', 'hyperbolic sine', 'cubic', 'hyperbolic sine + cubic']
    selected_function = tk.StringVar(window)
    selected_function.set("Select a function")
    torque_function_list = tk.OptionMenu(torque_function_frame, selected_function, *function_list,
                                         command=option_list_event)
    torque_function_list.pack(fill=tk.X)
    torque_figure = Figure()
    torque_canvas = FigureCanvasTkAgg(torque_figure, master=torque_function_frame)
    torque_canvas.draw()
    torque_canvas.get_tk_widget().pack(fill=tk.BOTH)

    button_frame = tk.Frame(master=window)
    button_frame.grid(row=2, column=0, padx=5, pady=5)
    send_button = tk.Button(master=button_frame, text="SEND", command=button_send_action)
    send_button.pack(fill=tk.BOTH)
    exit_button = tk.Button(master=button_frame, text="EXIT", command=window.destroy)
    exit_button.pack(fill=tk.BOTH)

    window.columnconfigure(0, weight=1)
    window.rowconfigure(0, weight=1)
    window.rowconfigure(1, weight=1)
    window.rowconfigure(2, weight=1)

    window.mainloop()
