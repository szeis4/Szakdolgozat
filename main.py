import serial
import math
from collections import deque
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import tkinter as tk
from tkinter import messagebox

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


def send_to_port(port_name, send, mode):
    try:
        pos_values = []
        tor_values = []
        ser = serial.Serial(port_name)
        ser.baudrate = 115200
        if mode == 1:
            for data in send:
                ser.write(data)
        elif mode == 2:
            data_for_read = [0x2c, 0x4d, 0x01, 0x00, 0x01, 255, 128]
            for i in range(-180, 180):
                data_for_read.append(0)
            ser.write(data_for_read)
            for i in range(500):
                read_data = ser.read(15)
                deg_top = read_data[7]
                deg_low = read_data[8]
                tor_top = read_data[11]
                tor_low = read_data[12]
                deg = int((deg_top << 8) + deg_low)
                tor = int((tor_top << 8) + tor_low)
                deg = deg - 180
                pos_values.append(deg)
                tor_values.append(tor)
            t = np.arange(0, len(pos_values), 1)
            position_figure.clear()
            read_torque_figure.clear()
            position_figure.add_subplot(111, title="Position").plot(t, pos_values)
            read_torque_figure.add_subplot(111, title="Torque").plot(t, tor_values)
            position_canvas.draw()
            read_torque_canvas.draw()
        elif mode == 3:
            for data in send:
                ser.write(data)
                read_data = ser.read(15)
                deg_top = read_data[7]
                deg_low = read_data[8]
                tor_top = read_data[11]
                tor_low = read_data[12]
                deg = int((deg_top << 8) + deg_low)
                tor = int((tor_top << 8) + tor_low)
                deg = deg - 180
                pos_values.append(deg)
                tor_values.append(tor)
            t = np.arange(0, len(pos_values), 1)
            position_figure.add_subplot(111, title="Position").plot(t, pos_values)
            read_torque_figure.add_subplot(111, title="Torque").plot(t, tor_values)
            position_canvas.draw()
            read_torque_canvas.draw()
        ser.close()
    except serial.serialutil.SerialException:
        messagebox.showinfo("Error", port_name + " port is not opened.")


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
    for i in range(0, len(stopping_points) - 1):
        if int(stopping_points[i]) > int(stopping_points[i + 1]):
            step_dir = -1
        else:
            step_dir = 1
        for j in range(int(stopping_points[i]), int(stopping_points[i + 1]), step_dir):
            movement_trajectory.append(j)
    return movement_trajectory


def button_send_action():
    if radiobutton_var.get() == 1 or radiobutton_var.get() == 2 or radiobutton_var.get() == 3:

        if selected_function.get() == "Select a function":

            messagebox.showinfo("Error", "Select a trajectory function!")

        else:
            trajectory = create_trajectory(trajectory_entry.get())

            movement_data = movement_data_create(trajectory, calculated_torques, int(amplitude_entry.get()),
                                                 int(frequency_entry.get()))

            send_data = to_byte_data(movement_data)

            send_to_port("COM3", send_data, radiobutton_var.get())

    else:

        messagebox.showinfo("Error", "Select an operation mode!")
        print(radiobutton_var)


if __name__ == '__main__':
    window = tk.Tk()
    window.title("Motor driver")

    left_frame = tk.Frame(master=window)
    left_frame.grid(row=0, column=0, padx=5, pady=5)

    mode_selection_frame = tk.Frame(master=left_frame)
    mode_selection_frame.grid(row=0, column=0, padx=5, pady=5)

    selection_label = tk.Label(master=mode_selection_frame, text="Select the mode")
    selection_label.grid(row=0, column=0, padx=5, pady=5)

    radiobutton_frame = tk.Frame(master=mode_selection_frame)
    radiobutton_frame.grid(row=1, column=0, padx=5, pady=5)

    radiobutton_var = tk.IntVar(window)

    write_radiobutton = tk.Radiobutton(master=radiobutton_frame, text="write", value=1, variable=radiobutton_var)
    write_radiobutton.grid(row=0, column=0, padx=5, pady=5)
    read_radiobutton = tk.Radiobutton(master=radiobutton_frame, text="read", value=2, variable=radiobutton_var)
    read_radiobutton.grid(row=0, column=1, padx=5, pady=5)
    read_write_radiobutton = tk.Radiobutton(master=radiobutton_frame, text="read-write", value=3,
                                            variable=radiobutton_var)
    read_write_radiobutton.grid(row=0, column=2, padx=5, pady=5)

    set_value_frame = tk.Frame(master=left_frame)
    set_value_frame.grid(row=1, column=0, padx=5, pady=5)

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

    torque_function_frame = tk.Frame(master=left_frame)
    torque_function_frame.grid(row=2, column=0, padx=5, pady=5)
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

    button_frame = tk.Frame(master=left_frame)
    button_frame.grid(row=3, column=0, padx=5, pady=5)
    send_button = tk.Button(master=button_frame, text="OPEN PORT", command=button_send_action)
    send_button.pack(fill=tk.BOTH)
    exit_button = tk.Button(master=button_frame, text="EXIT", command=window.destroy)
    exit_button.pack(fill=tk.BOTH)

    left_frame.columnconfigure(0, weight=1)
    left_frame.rowconfigure(0, weight=1)
    left_frame.rowconfigure(1, weight=1)
    left_frame.rowconfigure(2, weight=1)
    left_frame.rowconfigure(3, weight=1)

    right_frame = tk.Frame(master=window)
    right_frame.grid(row=0, column=1, padx=5, pady=5)

    right_column_label = tk.Label(master=right_frame, text="Figures of the read data")
    right_column_label.grid(row=0, column=0, padx=5, pady=5)

    position_figure = Figure()
    position_canvas = FigureCanvasTkAgg(position_figure, master=right_frame)
    position_canvas.draw()
    position_canvas.get_tk_widget().grid(row=1, column=0, padx=5, pady=5)

    read_torque_figure = Figure()
    read_torque_canvas = FigureCanvasTkAgg(read_torque_figure, master=right_frame)
    read_torque_canvas.draw()
    read_torque_canvas.get_tk_widget().grid(row=2, column=0, padx=5, pady=5)

    window.mainloop()
