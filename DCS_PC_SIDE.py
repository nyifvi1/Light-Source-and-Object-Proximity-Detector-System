import serial
import threading
import time
import numpy as np
import os
from math import tan, pi, cos, sin, radians
import tkinter as tk
from scipy.interpolate import interp1d
from scipy.signal import argrelextrema
from scipy.signal import find_peaks
from tkinter import simpledialog, messagebox

SERIAL_PORT = 'COM3'  #need to be edited
BAUD_RATE = 9600
lock = threading.Lock()
samples = np.zeros((180, 4), dtype=int)  #index is angle, 0 col is sample value of Ultrasonic, 1 col is distance cal,
LDR_calib_arr = np.zeros((10,2), dtype=int)  #first col is LDR0, second col is LDR1, index is 5,10,15,..,50
# -----------------------------GUI functions-------------------------------
def launch_main_gui():
    menu_window = tk.Tk()
    menu_window.title("MCU Control Menu")
    menu_window.geometry("400x600")
    menu_window.configure(bg='#DDF6FF')
    tk.Label(menu_window, text="=== MENU ===", font=("Segoe UI", 14, "bold"), bg='#DDF6FF').pack(pady=15)
    tk.Button(menu_window, text="0. Sleep", command=lambda: [menu_window.quit(), sleep_mode()]).pack(pady=5)
    tk.Button(menu_window, text="1. Objects Detector", command=lambda: [menu_window.quit(), objects_detector()]).pack(pady=5)
    tk.Button(menu_window, text="2. Telemeter", command=lambda: [menu_window.quit(), telemeter()]).pack(pady=5)
    tk.Button(menu_window, text="3. Light Source Detector", command=lambda: [menu_window.quit(), light_source_detector()]).pack(pady=5)
    tk.Button(menu_window, text="4. Objects and Light Combined", command=lambda: [menu_window.quit(), combined_mode()]).pack(pady=5)
    tk.Button(menu_window, text="5. Send and Store Files", command=lambda: [menu_window.quit(), send_and_store_files()]).pack(pady=5)
    tk.Button(menu_window, text="6. LDR Calibration", command=lambda: [menu_window.quit(), calibrate_ldr()]).pack(pady=5)
    tk.Button(menu_window, text="7. Reset Flash Memory", command=lambda: [menu_window.quit(), reset_flash_segments()]).pack(pady=5)
    tk.Button(menu_window, text="8. Read Text Files", command=lambda: [menu_window.quit(), read_text_files()]).pack(pady=5)
    tk.Button(menu_window, text="9. Run Script Files", command=lambda: [menu_window.quit(), run_script_files()]).pack(pady=5)
    menu_window.mainloop()
    menu_window.destroy()

def sleep_mode():
    global state
    print("-------------------------------Switched to Sleep Mode-------------------------------")
    state = "0"

def objects_detector():
    global state
    print("-------------------------------Launching Objects Detector Mode...-------------------------------")
    state = "1"

def telemeter():
    global state
    print("-------------------------------Launching Telemeter Mode...-------------------------------")
    state = "2"

def light_source_detector():
    global state
    print("-------------------------------Launching Light Source Detector Mode...-------------------------------")
    state = "3"

def combined_mode():
    global state
    print("-------------------------------Launching Combined Mode...-------------------------------")
    state = "4"

def send_and_store_files():
    global  state
    print("-------------------------------Launching File Sender Mode...-------------------------------")
    state = "5"

def calibrate_ldr():
    global state,sample_index
    print("-------------------------------Launching LDR Calibration Mode...-------------------------------")
    sample_index = 0
    state = "6"

def reset_flash_segments():
    global state
    print("-------------------------------Resetting Flash Memory Segments...-------------------------------")
    state = "7"

def read_text_files():
    global state
    print("-------------------------------Reading Text Files from MCU...-------------------------------")
    state = "8"

def run_script_files():
    global state
    print("-------------------------------Running Script Files on MCU...-------------------------------")
    state = "9"
# ----------------------------------GUI auxiliary functions-----------------------
def launch_angle_input_gui():
    root = tk.Tk()
    root.withdraw()
    while True:
        angle = simpledialog.askinteger("Telemeter Input", "Enter angle (0–179):", minvalue=0, maxvalue=179)
        if angle is None:
            messagebox.showinfo("Canceled", "Operation canceled by user.")
            return None
        if 0 <= angle <= 179:
            return str(angle)
        else:
            messagebox.showerror("Invalid Input", "Please enter a value between 0 and 179.")

def exit_telemeter_window():
    # Create the main window
    window = tk.Tk()
    window.title("Telemeter")
    window.geometry("300x150")

    # Create and place the Exit button
    exit_button = tk.Button(window, text="Exit Telemeter", command=window.quit)
    exit_button.pack(expand=True)

    # Start the GUI loop and block until the window is closed
    window.mainloop()
    window.destroy()

def launch_threshold_input_gui():
    root = tk.Tk()
    root.withdraw()  # Hide main window
    while True:
        distance_threshold = simpledialog.askinteger("Distance Threshold Input", "Enter threshold (0–400):", minvalue=0, maxvalue=400)
        if distance_threshold is None:
            messagebox.showinfo("Canceled", "Operation canceled by user.")
            return None
        if 0 <= distance_threshold <= 400:
            return str(distance_threshold)
        else:
            messagebox.showerror("Invalid Input", "Please enter a value between 0 and 400.")

def get_file_type():
    result = {'choice': None}
    def choose_script():
        result['choice'] = 's'
        window.quit()
    def choose_text():
        result['choice'] = 't'
        window.quit()
    window = tk.Tk()
    window.title("Choose File Type")
    window.geometry("300x150")
    label = tk.Label(window, text="Press S for script file or T for text file")
    label.pack(pady=10)
    button_s = tk.Button(window, text="S - Script", command=choose_script)
    button_s.pack(pady=5)
    button_t = tk.Button(window, text="T - Text", command=choose_text)
    button_t.pack(pady=5)
    window.mainloop()
    window.destroy()
    return result['choice']

def get_file_path():
    root = tk.Tk()
    root.withdraw()  # Hide the main window

    file_path = simpledialog.askstring("File Path Input", "Please enter the file path:")
    root.destroy()
    return file_path
# -----------------------------auxiliary functions-------------------------------
def encode_commands(script_text):
    opcodes = {
        "inc_lcd": 0x01,
        "dec_lcd": 0x02,
        "rra_lcd": 0x03,
        "set_delay": 0x04,
        "clear_lcd": 0x05,
        "servo_deg": 0x06,
        "servo_scan": 0x07,
        "sleep": 0x08
    }
    out_chars = []
    for raw_line in script_text.splitlines():
        s = raw_line.strip()
        if not s:
            continue
        if " " in s:
            cmd, params = s.split(" ", 1)
            nums = [int(x) & 0xFF for x in params.replace(" ", "").split(",") if x != ""]
        else:
            cmd, nums = s, []
        code = opcodes.get(cmd)
        if code is None:
            continue
        out_chars.append(chr(code & 0xFF))
        for n in nums:
            out_chars.append(chr(n & 0xFF))
    return "".join(out_chars)

def visualize_objects(ultraobjects=None, light_objects=None, distance_threshold=55):
    window = tk.Tk()
    window.title("Object Visualization")
    canvas_size = 600
    canvas = tk.Canvas(window, width=canvas_size, height=canvas_size, bg="white")
    canvas.pack()
    center_x = canvas_size // 2
    center_y = canvas_size // 2
    canvas.create_arc(center_x - 250, center_y - 250, center_x + 250, center_y + 250,
                      start=0, extent=180, style=tk.ARC)
    for angle in range(0, 181, 10):
        angle_rad = radians(angle)
        x = center_x + 260 * cos(angle_rad)
        y = center_y - 260 * sin(angle_rad)
        canvas.create_text(x, y, text=str(angle), font=("Arial", 7), fill="gray")
    scale = 250 / distance_threshold
    if ultraobjects:
        for obj in ultraobjects:
            angle_deg = obj['center_angle']
            distance = obj['average_distance']
            length = obj['length_cm']
            r = distance * scale
            angle_rad = radians(angle_deg)
            x = center_x + r * cos(angle_rad)
            y = center_y - r * sin(angle_rad)
            canvas.create_oval(x-5, y-5, x+5, y+5, fill="red")
            label = f"Angle: {angle_deg}°\nDist: {int(distance)}cm\nLen: {length:.1f}cm"
            canvas.create_text(x, y-25, text=label, font=("Arial", 8), fill="black")
    if light_objects:
        for obj in light_objects:
            angle_deg = obj[0]
            distance = obj[2]
            if distance is None:
                continue
            r = distance * scale
            angle_rad = radians(angle_deg)
            x = center_x + r * cos(angle_rad)
            y = center_y - r * sin(angle_rad)
            canvas.create_oval(x-5, y-5, x+5, y+5, fill="yellow")
            label = f"Angle: {angle_deg}°\nDist: {int(distance)}cm"
            canvas.create_text(x, y-25, text=label, font=("Arial", 8), fill="black")
    exit_button = tk.Button(window, text="Exit", command=window.quit, font=("Arial", 12), bg="lightgray")
    exit_button.pack(pady=10)
    window.mainloop()
    window.destroy()

def correction_factor(distance, min_dist=10, max_dist=50, min_factor=0.4, max_factor=2.2):  ###calibrate
    a = 3.0
    b = 0.064
    c = 0.4
    return a * np.exp(-b * distance) + c

def detect_objects(samples, distance_threshold=55, jump_threshold=15, merge_angle_thresh=20, merge_distance_thresh=20):
    distances = samples[:, 1]
    angles = np.arange(len(distances))
    objects = []
    current_object = []
    for i, (angle, distance) in enumerate(zip(angles, distances)):
        if distance < distance_threshold:
            if current_object and abs(distance - current_object[-1][1]) > jump_threshold:
                angles_obj = [item[0] for item in current_object]
                dists_obj = [item[1] for item in current_object]
                center_angle = angles_obj[len(angles_obj) // 2]
                average_distance = sum(dists_obj) / len(dists_obj)
                angular_length = angles_obj[-1] - angles_obj[0] + 1
                #factor = length_correction_factor(average_distance)
                #factor = correction_factor(average_distance)
                factor =  correction_factor(average_distance)
                length_cm = 2 * average_distance * tan((angular_length / 2) * pi / 180) * factor
                #length_cm = 2 * average_distance * tan((angular_length / 2) * pi / 180)
                objects.append({
                    "center_angle": center_angle,
                    "average_distance": average_distance,
                    "length_cm": length_cm
                })
                current_object = []
            current_object.append((angle, distance))
        else:
            if current_object:
                angles_obj = [item[0] for item in current_object]
                dists_obj = [item[1] for item in current_object]
                center_angle = angles_obj[len(angles_obj) // 2]
                average_distance = sum(dists_obj) / len(dists_obj)
                angular_length = (angles_obj[-1] - angles_obj[0] + 1) ###calibrate
                #print(angular_length)
                factor = correction_factor(average_distance)
                #length_cm = 2 * average_distance * tan((angular_length / 2) * pi / 180) * factor
                length_cm = 2 * average_distance * tan((angular_length / 2) * pi / 180) * factor
                objects.append({
                    "center_angle": center_angle,
                    "average_distance": average_distance,
                    "length_cm": length_cm
                })
                current_object = []
    if current_object:
        angles_obj = [item[0] for item in current_object]
        dists_obj = [item[1] for item in current_object]
        center_angle = angles_obj[len(angles_obj) // 2]
        average_distance = sum(dists_obj) / len(dists_obj)
        angular_length = angles_obj[-1] - angles_obj[0] + 1
        factor = correction_factor(average_distance)
        length_cm = 2 * average_distance * tan((angular_length / 2) * pi / 180) * factor
        objects.append({
            "center_angle": center_angle,
            "average_distance": average_distance,
            "length_cm": length_cm
        })
    merged_objects = []
    for obj in objects:
        if not merged_objects:
            merged_objects.append(obj)
        else:
            last = merged_objects[-1]
            if abs(obj["center_angle"] - last["center_angle"]) <= merge_angle_thresh and \
               abs(obj["average_distance"] - last["average_distance"]) <= merge_distance_thresh:

                new_center = (obj["center_angle"] + last["center_angle"]) // 2
                new_distance = (obj["average_distance"] + last["average_distance"]) / 2
                new_length = obj["length_cm"] + last["length_cm"]
                merged_objects[-1] = {
                    "center_angle": new_center,
                    "average_distance": new_distance,
                    "length_cm": new_length
                }
            else:
                merged_objects.append(obj)
    return merged_objects

def detect_light_source(LDR_calib_arr, sample):
    avg_calib = np.mean(LDR_calib_arr, axis=1)
    distances = np.arange(5, 55, 5)
    calib_diff = np.abs(LDR_calib_arr[:, 0] - LDR_calib_arr[:, 1])
    thresholds = calib_diff / avg_calib
    sample_diff = abs(sample[0] - sample[1])
    sample_avg = (sample[0] + sample[1]) / 2
    threshold_interp = interp1d(avg_calib, thresholds, bounds_error=False, fill_value="extrapolate")
    dynamic_threshold = threshold_interp(sample_avg)
    if sample_diff / sample_avg <= dynamic_threshold*1.2:  #need to be calibrated
        distance_interp = interp1d(avg_calib, distances, bounds_error=False, fill_value="extrapolate")
        estimated_distance = distance_interp(sample_avg)
        if estimated_distance <= 55:
            return True, round(float(estimated_distance), 2)
    return False, None

def detect_light_objects(LDR_calib_arr, samples, min_distance_between_objects=20):
    results = []
    distances = []
    for i in range(samples.shape[0]):
        sample = samples[i, 2:4]
        is_perpendicular, distance = detect_light_source(LDR_calib_arr, sample)
        results.append((i, is_perpendicular, distance))
        distances.append(distance if distance is not None else np.inf)
    distances = np.array(distances)
    valid_indices = np.where(distances < np.inf)[0]
    valid_distances = distances[valid_indices]
    local_minima_indices = argrelextrema(valid_distances, np.less)[0]
    filtered_minima = []
    for idx in local_minima_indices:
        if not filtered_minima or idx - filtered_minima[-1] >= min_distance_between_objects:
            filtered_minima.append(idx)
    final_results = []
    for idx in filtered_minima:
        original_index = valid_indices[idx]
        final_results.append(results[original_index])
    return final_results

def visualize_objects_servo_scan(ultraobjects, left_angle=0, right_angle=180, distance_threshold=55):
    window = tk.Tk()
    window.title("UltraObject Visualization")
    canvas_size = 600
    canvas = tk.Canvas(window, width=canvas_size, height=canvas_size, bg="white")
    canvas.pack()
    center_x = canvas_size // 2
    center_y = canvas_size // 2
    canvas.create_arc(center_x - 250, center_y - 250, center_x + 250, center_y + 250,
                      start=0, extent=180, style=tk.ARC)
    for angle in range(left_angle, right_angle + 1, 10):
        angle_rad = radians(angle)
        x = center_x + 260 * cos(angle_rad)
        y = center_y - 260 * sin(angle_rad)
        canvas.create_text(x, y, text=str(angle), font=("Arial", 7), fill="gray")
    scale = 250 / distance_threshold
    for obj in ultraobjects:
        angle_deg = obj['center_angle']
        if not (left_angle <= angle_deg <= right_angle):
            continue
        distance = obj['average_distance']
        length = obj['length_cm']
        r = distance * scale
        angle_rad = radians(angle_deg)
        x = center_x + r * cos(angle_rad)
        y = center_y - r * sin(angle_rad)
        canvas.create_oval(x-5, y-5, x+5, y+5, fill="red")
        label = f"Angle: {angle_deg}°\nDist: {int(distance)}cm\nLen: {length:.1f}cm"
        canvas.create_text(x, y-25, text=label, font=("Arial", 8), fill="black")
    exit_button = tk.Button(window, text="Exit", command=window.quit, font=("Arial", 12), bg="lightgray")
    exit_button.pack(pady=10)
    window.mainloop()
    window.destroy()
# -----------------------------reading thread-------------------------------
def read_from_serial(ser):
    global state,data,MCU_got_chunk,st9_unlock,st9_servo_deg_flag,st9_servo_scan_flag,sample_index,angle,samples,distance_threshold_input,left_servo_scan_angle
    while True:
        if ser.in_waiting:
            with lock:
                # ----------------------------------------------------------------------
                if state == "0":
                    ser.write(('T' + '$$').encode('utf-8'))  # UART transfer was successful
                    ser.reset_input_buffer()
                # ----------------------------------------------------------------------
                if state == "1" or state == "2" or st9_servo_deg_flag == 1 or st9_servo_scan_flag == 1:
                    ser.timeout = 1
                    data = ser.read(4)
                    if len(data) == 4:
                        if ((data[0] + data[1] + data[2] + data[3]) % 256) == 0:
                            ser.write(('T' + '$$').encode('utf-8'))  #UART transfer was successful
                            angle = data[0]
                            lsb = data[1]
                            msb = data[2]
                            if left_servo_scan_angle == -1:
                                left_servo_scan_angle = angle
                            samples[angle, 0] = (msb << 8) | lsb                               #sample value
                            samples[angle, 1] = (samples[angle, 0]) * (2 ** -20) * (34645 / 2) #distance calc
                            print(f"Angle: {angle}°, Sample: {samples[angle, 0]}, Distance: {samples[angle, 1]} cm")
                            if angle == 179:
                                angle += 1
                        else:
                            ser.write(('F' + '$$').encode('utf-8')) #UART transfer was unsuccessful
                            print("UART transfer was unsuccessful1")
                            ser.reset_input_buffer()
                    elif  len(data) == 2:
                        if ((data[0]+data[1]) % 256) == 0:
                            if data[0] == ord("e"):
                                right_servo_scan_angle = angle
                                np.savetxt("samples.txt", samples, fmt="%d")
                                ultrasonic_objects = detect_objects(samples,distance_threshold=distance_threshold_input)
                                visualize_objects_servo_scan(ultraobjects=ultrasonic_objects,left_angle=left_servo_scan_angle,right_angle=right_servo_scan_angle,distance_threshold=distance_threshold_input)
                                st9_servo_scan_flag = 0  # +close servo_scan GUI
                                ser.write(('S' + '$$').encode('utf-8'))
                    else:
                        ser.write(('F' + '$$').encode('utf-8'))  # UART transfer was unsuccessful
                        print("UART transfer was unsuccessful2")
                        ser.reset_input_buffer()
                # ----------------------------------------------------------------------
                if state == "3":
                    ser.timeout = 1
                    data = ser.read(5)
                    ser.timeout = 1
                    ser.reset_input_buffer()
                    if len(data) == 5:
                        if ((data[0] + data[1] + data[2] + data[3]+data[4]) % 256) == 0:
                            ser.write(('T' + '$$').encode('utf-8'))  #UART transfer was successful
                            angle = data[0]
                            LDR0_val = data[1] | ((data[2] & 0x03) << 8)
                            LDR1_val = ((data[2] >> 2) & 0x3F) | (data[3] << 6)
                            samples[angle, 2] = LDR0_val
                            samples[angle, 3] = LDR1_val
                            print(f"Angle: {angle}°, LDR0: {LDR0_val}, LDR1: {LDR1_val}")
                            if angle == 179:
                                angle += 1
                        else:
                            ser.write(('F' + '$$').encode('utf-8')) #UART transfer was unsuccessful
                            print("UART transfer was unsuccessful1")
                            ser.reset_input_buffer()
                    else:
                        ser.write(('F' + '$$').encode('utf-8'))  # UART transfer was unsuccessful
                        print("UART transfer was unsuccessful2")
                        ser.reset_input_buffer()
                # ----------------------------------------------------------------------
                if state == "4":
                    data = ser.read(7)
                    ser.reset_input_buffer()
                    if len(data) == 7:
                        if ((data[0] + data[1] + data[2] + data[3]+data[4]+data[5]+data[6]) % 256) == 0:
                            ser.write(('T' + '$$').encode('utf-8'))  #UART transfer was successful
                            angle = data[0]
                            lsb = data[1]  # ultrasonic
                            msb = data[2]  # ultrasonic
                            LDR0_val = data[3] | ((data[4] & 0x03) << 8)
                            LDR1_val = ((data[4] >> 2) & 0x3F) | (data[5] << 6)
                            samples[angle, 0] = (msb << 8) | lsb                               #sample value
                            samples[angle, 1] = (samples[angle, 0]) * (2 ** -20) * (34645 / 2) #distance calc
                            samples[angle, 2] = LDR0_val
                            samples[angle, 3] = LDR1_val
                            print(f"Angle: {angle}°, Sample: {samples[angle, 0]}, Distance: {samples[angle, 1]} cm LDR0: {LDR0_val}, LDR1: {LDR1_val}")
                            if angle == 179:
                                angle += 1
                        else:
                            ser.write(('F' + '$$').encode('utf-8')) #UART transfer was unsuccessful
                            print("UART transfer was unsuccessful1")
                            ser.reset_input_buffer()
                    else:
                        ser.write(('F' + '$$').encode('utf-8'))  # UART transfer was unsuccessful
                        print("UART transfer was unsuccessful2")
                        ser.reset_input_buffer()
                # ----------------------------------------------------------------------
                if state == "5":
                    data=ser.read(2)
                    ser.reset_input_buffer()
                    if len(data) == 2:
                        if ((data[0] + data[1]) % 256) == 0:
                            if data[0] == ord("a"):
                                MCU_got_chunk = 1
                            else:
                                print("get ack from MCU was unsuccessful2")
                    else:
                        print("get ack from MCU was unsuccessful")
                # ----------------------------------------------------------------------
                if state == "6" or state == "10":
                    data = ser.read(5)
                    ser.reset_input_buffer()
                    if len(data) == 5:
                        LDR0_val = data[1] | ((data[2] & 0x03) << 8)
                        LDR1_val = ((data[2] >> 2) & 0x3F) | (data[3] << 6)
                        if ((data[0] + data[1] + data[2] + data[3]+data[4]) % 256) == 0:
                            ser.write(('T' + '$$').encode('utf-8'))  #UART transfer was successful
                            sample_index = data[0] - 1
                            print(f"LDR0 calibration: {LDR0_val}, LDR1 calibration: {LDR1_val}, Sample distance: {5 * (sample_index + 1)} cm")
                            LDR_calib_arr[sample_index,0] = LDR0_val
                            LDR_calib_arr[sample_index,1] = LDR1_val
                            if sample_index == 9:
                                sample_index += 1
                        else:
                            ser.write(('F' + '$$').encode('utf-8')) #UART transfer was unsuccessful
                            print("UART transfer was unsuccessful1")
                            ser.reset_input_buffer()
                    else:
                        ser.write(('F' + '$$').encode('utf-8'))  # UART transfer was unsuccessful
                        print("UART transfer was unsuccessful2")
                        ser.reset_input_buffer()
                # ----------------------------------------------------------------------
                if state == "9" and st9_servo_deg_flag == 0 and st9_servo_scan_flag == 0:
                    data = ser.read(2)
                    if len(data) == 2:
                        if ((data[0]+data[1]) % 256) == 0:
                            if data[0] == ord("f"):
                                st9_unlock = 1
                            elif data[0] == ord("d"):
                                st9_servo_deg_flag = 1
                            elif data[0] == ord("s"):
                                st9_servo_scan_flag = 1   #+open servo_scan GUI
                                left_servo_scan_angle = -1
                                ser.write(('U' + '$$').encode('utf-8'))
                        else:
                            print("get ack from MCU was unsuccessful2 9")
                    else:
                        print("get ack from MCU was unsuccessful 9")
# -----------------------------------------------Main FSM-----------------------------------------------------
def main():
    global state,MCU_got_chunk,st9_unlock,st9_servo_deg_flag,st9_servo_scan_flag,sample_index,angle,samples,distance_threshold_input,left_servo_scan_angle
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        thread = threading.Thread(target=read_from_serial, args=(ser,), daemon=True)
        thread.start()

        st9_unlock = 0
        st9_servo_deg_flag = 0
        st9_servo_scan_flag = 0
        sample_index = 0
        angle = 0
        state = "0"
        distance_threshold_input = 55
        left_servo_scan_angle = -1
        #-----------sync LDR0,LDR1 values from flash----------
        time.sleep(2)
        state = "10"
        ser.write(('A' + '$$').encode('utf-8'))
        while sample_index != 10:
            time.sleep(0.01)
        state = "0"
        np.savetxt("LDR_calib.txt", LDR_calib_arr, fmt="%d")
        # ---------------------------------------------------
        while True:
            if state == "0":
                launch_main_gui()
                ser.write((state + '$$').encode('utf-8'))
            # ---------------------------------------------------
            if state == "1":
                distance_threshold_input = int(launch_threshold_input_gui())
                if distance_threshold_input is not None:
                    print("User selected distance threshold:", distance_threshold_input)
                else:
                    print("User canceled input.")
                while True:
                    if angle == 180:
                        break
                    time.sleep(0.01)
                with lock:
                    angle = 0
                np.savetxt("samples.txt", samples, fmt="%d")
                ultrasonic_objects = detect_objects(samples, distance_threshold=distance_threshold_input)
                visualize_objects(ultraobjects=ultrasonic_objects, distance_threshold=distance_threshold_input)
                state = "0"
            # ---------------------------------------------------
            if state == "2":
                angle_input = launch_angle_input_gui()
                if angle_input is not None:
                    print("User selected angle:", angle_input)
                    ser.write((angle_input + '$$').encode('utf-8'))
                else:
                    print("User canceled input.")
                exit_telemeter_window()
                state = "0"
                ser.write((state + '$$').encode('utf-8'))
            # ---------------------------------------------------
            if state == "3":
                while True:
                    if angle == 180:
                        break
                    time.sleep(0.01)
                with lock:
                    angle = 0
                np.savetxt("samples.txt", samples, fmt="%d")
                light_objects = detect_light_objects(LDR_calib_arr,samples)
                visualize_objects(light_objects=light_objects, distance_threshold=55)
                state = "0"
            # ---------------------------------------------------
            if state == "4":
                distance_threshold_input = int(launch_threshold_input_gui())
                if distance_threshold_input is not None:
                    print("User selected distance threshold:", distance_threshold_input)
                else:
                    print("User canceled input.")
                while True:
                    if angle == 180:
                        break
                    time.sleep(0.01)
                with lock:
                    angle = 0
                np.savetxt("samples.txt", samples, fmt="%d")
                ultrasonic_objects = detect_objects(samples, distance_threshold=distance_threshold_input)
                light_objects = detect_light_objects(LDR_calib_arr,samples)
                visualize_objects(ultraobjects=ultrasonic_objects,light_objects=light_objects, distance_threshold=distance_threshold_input)
                state = "0"
            # ---------------------------------------------------
            if state == "5":
                file_type = get_file_type()
                ser.write((file_type + '$$').encode('utf-8'))
                path = get_file_path()
                filename = os.path.basename(path)
                with lock:
                    MCU_got_chunk = 0
                filename = filename.ljust(16)              #padding name to 16 chars
                ser.write((filename + '$$').encode('utf-8'))
                while True:
                    if MCU_got_chunk == 1:
                        break
                    time.sleep(0.01)
                with open(path, 'r', encoding='utf-8') as f:
                    file_data = f.read()
                if file_type == 's':
                    file_data = encode_commands(file_data)
                chunk_size = 19
                index = 0
                while index < len(file_data):
                    chunk = file_data[index:index + chunk_size]
                    if index+chunk_size >= len(file_data):
                        send_more_n = 0
                        if len(chunk)<=17:
                            chunk = chunk + "$$"
                        elif len(chunk) == 18:
                            chunk = chunk + "$$"
                        elif len(chunk) == 19:
                            send_more_n = 1
                    with lock:
                        MCU_got_chunk = 0
                    ser.reset_output_buffer()
                    if file_type == 's':
                        ser.write(chunk.encode('latin-1'))
                    else:
                        ser.write(chunk.encode('utf-8'))
                    while True:
                        if MCU_got_chunk == 1:
                            break
                        time.sleep(0.01)
                    index += chunk_size
                if send_more_n == 1:
                    with lock:
                        MCU_got_chunk = 0
                    ser.write("$$".encode('utf-8'))
                    while True:
                        if MCU_got_chunk == 1:
                            break
                        time.sleep(0.01)
                print("file loaded successfully")
                state = "0"
            # ---------------------------------------------------
            if state == "6":
                while True:
                    if sample_index == 10:
                        break
                    time.sleep(0.01)
                print("Calibration of LDR0 and LDR1 sensors was completed successfully and values have been stored")
                state = "0"
            # ---------------------------------------------------
            if state == "7":
                print("Memory flash segments associated with File Mode have been successfully reset")
                state = "0"
            # ---------------------------------------------------
            if state == "8":
                print("The MCU is now ready for reading stored text files")
                state = "0"
            # ---------------------------------------------------
            if state == "9":
                print("You can now select script files stored on the MCU")
                with lock:
                    st9_unlock = 0
                    st9_servo_deg_flag = 0
                    st9_servo_scan_flag = 0
                while True:
                    if st9_servo_deg_flag == 1:
                        #servo_deg_user_input = input("Press 'f' to finish servo_deg")
                        exit_telemeter_window()
                        with lock:
                            st9_servo_deg_flag = 0
                        ser.write(('D' + '$$').encode('utf-8'))  #finish servo_deg in MCU side
                    if st9_unlock == 1:
                        break
                    time.sleep(0.01)
                print("selected script file execution has finished")
                state = "0"
            # ---------------------------------------------------
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


if __name__ == '__main__':
    main()



