import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import serial
import os
import time
from ultralytics import YOLO
import cv2
import numpy as np
from chessboard import *

# Configure the serial port
ser = serial.Serial('COM7', 9600)  # Update the port and baud rate accordingly

camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Use the first camera (index 0)
camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Load trained model
model_path = os.path.join('.', 'v14best.pt')
model = YOLO(model_path)
threshold = 0.5

colors = [(255, 0, 0), (255, 255, 0), (0, 255, 255), (255, 0, 255), (0, 255, 0), (0, 0, 255), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255), (255, 0, 0)]


# Used to save mp4 videos
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
videout = cv2.VideoWriter('outputv132.mp4', fourcc, 24.0, (640, 480))

# Used to store chess board corner positions on startup.
corners = None
frame = None

# Used to calculate fps
lastTime = time.time_ns()
dt = time.time_ns() - lastTime
def update_camera_view():
    global lastTime
    dt = time.time_ns() - lastTime
    lastTime += dt
    fps = int(1e9/dt)
    
    global frame
    _, frame = camera.read()

    if frame is not None:

        # FPS
        cv2.putText(frame, f'{fps}', (15, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 1, cv2.LINE_AA)

        pieces = []
        # Get model results higher than threshold
        results = model(frame)[0]
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score > threshold:
                pieces.append([int(class_id), ((x2+x1)*0.5 , (0.667*y2+0.333*y1))])
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), colors[int(class_id)], 1)
                cv2.putText(frame, f'{results.names[int(class_id)].upper()[0:1]}{results.names[int(class_id)].upper()[6:]}', (int(x1), int(y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[int(class_id)], 1, cv2.LINE_AA)
        
        # Projection Transformation to get 2D board.
        global corners
        if corners is not None:
            board2D = extractBoard(frame, corners)
            print(corners[0][1])
            cv2.circle(frame, (int(corners[0][0]), int(corners[0][1])), 3, (0, 0, 255), -1)
            cv2.circle(frame, (int(corners[1][0]), int(corners[1][1])), 3, (0, 0, 255), -1)
            cv2.circle(frame, (int(corners[2][0]), int(corners[2][1])), 3, (0, 0, 255), -1)
            cv2.circle(frame, (int(corners[3][0]), int(corners[3][1])), 3, (0, 0, 255), -1)
            if len(pieces) > 0:
                board = boardimg.copy()
                w = 132
                h = 132

                matrix = getPerspectiveTransformationMatrix(corners)
                for piece in pieces:
                    p = transformPoint(piece[1], matrix)
                    pos = getSquareFromPoint(p)
                    if not (pos[0] < 0 or pos[0] > 7 or pos[1] < 0 or pos[1] > 7):
                        y_pos = (7-pos[1])*h
                        x_pos = pos[0]*w
                        print(pos)
                        sprite = pieceList[piece[0]]
                        board[y_pos:y_pos + sprite.shape[0], x_pos:x_pos + sprite.shape[1]] = sprite
                        cv2.circle(board2D, p, 5, colors[int(piece[0])], -1)
                
                board = cv2.resize(board, (640, 640))
                cv2.imshow("2D Board Chesscom", board)
            cv2.imshow("2D Board", board2D)
            cv2.waitKey(0)

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (640, 480))
        videout.write(frame)

        
        # Convert the frame to an image compatible with Tkinter
        img = Image.fromarray(frame)
        img = ImageTk.PhotoImage(image=img)
        
        # Update the label with the new image
        camera_label.configure(image=img)
        camera_label.image = img
    
    # Schedule the function to be called after 10 milliseconds
    root.after(33, update_camera_view)

# folder path
dir_path = 'images'
count = 0
# Iterate directory
for path in os.listdir(dir_path):
    # check if current path is a file
    if os.path.isfile(os.path.join(dir_path, path)):
        count += 1
count = 828
def save_screenshot():
    _, frame = camera.read()  # Read a frame from the camera
    frame = cv2.resize(frame, (640, 480))
    if frame is not None:
        # Generate a unique filename for the screenshot'
        global count
        screenshot_filename = os.path.join('images', f'screenshot{count}.png')
        count = count + 1
        
        # Save the screenshot
        cv2.imwrite(screenshot_filename, frame)

def image_label():
    _, frame = camera.read()
    frame = cv2.resize(frame, (640, 480))
    if frame is not None:
        # Generate a unique filename for the screenshot'
        global count
        screenshot_filename = os.path.join('extradata', 'images', f'screenshot{count}.png')
        

        txt_filename = os.path.join('extradata', 'labels', f'screenshot{count}.txt')
        file = open(txt_filename, "w")
        
        framecopy = frame.copy()
        results = model(frame)[0]
        pcs = 0
        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score > threshold:
                pcs += 1
                x = (x2+x1)/2/640
                y = (y2+y1)/2/480
                w = (x2-x1)/640
                h = (y2-y1)/480
                string = f'{int(class_id)} {x} {y} {w} {h}\n'
                file.write(string)
                cv2.rectangle(framecopy, (int(x1), int(y1)), (int(x2), int(y2)), colors[int(class_id)], 1)
                cv2.putText(framecopy, f'{results.names[int(class_id)].upper()[0:1]}{results.names[int(class_id)].upper()[6:]}', (int(x1), int(y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[int(class_id)], 1, cv2.LINE_AA)
        
        #cv2.imshow(f'Count: {pcs}', framecopy)
        #cv2.waitKey(0)
        cv2.imwrite(screenshot_filename, frame)
        count = count + 1

def delete_last():
    global count
    count -= 1
    filenameimg = os.path.join(".", "extradata", "images", f'screenshot{count}.png')
    filenametxt = os.path.join(".", "extradata", "labels", f'screenshot{count}.txt')

    os.remove(filenameimg)
    os.remove(filenametxt)

def find_chessboard():
    global corners
    global frame
    corners = getCornersFromImage(frame)

def send_custom_command():
    x_value = x_entry.get()
    y_value = y_entry.get()
    z_value = z_entry.get()
    t_value = t_entry.get()
    option_value = option_var.get()
    
    custom_command = f'X{x_value}Y{y_value}Z{z_value}T{t_value}O{option_value}'
    send_serial_command(custom_command)

def send_jerk_command():
    jerk_value = "J1" if jerk_var.get() else "J0"
    send_serial_command(jerk_value)
    
def send_grip_command():
    send_serial_command("J190J2125J3-90J40J5-100J60")
    #send_serial_command("grip")


def send_jog_command(motor, steps):
    jog_command = f'{motor}{steps}'
    send_serial_command(jog_command)

def send_calibration_command():
    send_serial_command("calibrate")

def send_serial_command(command):
    ser.write(command.encode('utf-8'))

# GUI
root = tk.Tk()
root.title("Motor Control and Custom Command Sender")

frame = ttk.Frame(root, padding="20")
frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

# X, Y, Z Entry
ttk.Label(frame, text="X:").grid(row=0, column=0, sticky=tk.W)
x_entry = ttk.Entry(frame)
x_entry.grid(row=0, column=1)

ttk.Label(frame, text="Y:").grid(row=1, column=0, sticky=tk.W)
y_entry = ttk.Entry(frame)
y_entry.grid(row=1, column=1)

ttk.Label(frame, text="Z:").grid(row=2, column=0, sticky=tk.W)
z_entry = ttk.Entry(frame)
z_entry.grid(row=2, column=1)

ttk.Label(frame, text="Time:").grid(row=3, column=0, sticky=tk.W)
t_entry = ttk.Entry(frame)
t_entry.grid(row=3, column=1)

# Option Dropdown
ttk.Label(frame, text="Option:").grid(row=4, column=0, sticky=tk.W)
options = ['LOOKRIGHT', 'LOOKLEFT', 'LOOKFORWARD', 'LOOKBACKWARDS', 'LOOKDOWN', 'LOOKUP']
option_var = tk.StringVar(value=options[0])
option_dropdown = ttk.Combobox(frame, textvariable=option_var, values=options)
option_dropdown.grid(row=4, column=1, padx=5, pady=5)

# Jog Motor Buttons
jog_frame = ttk.LabelFrame(root, text="Jog Motor")
jog_frame.grid(row=1, column=0, padx=20, pady=10)

motor_letters = ['A', 'B', 'C', 'D', 'E', 'F']
motor_steps = [-1000, -100, 100, 1000]

for motor, motor_letter in enumerate(motor_letters, start=1):
    ttk.Label(jog_frame, text=f"Motor {motor_letter}:").grid(row=motor-1, column=0, padx=5, pady=5)
    for step in motor_steps:
        jog_button = ttk.Button(jog_frame, text=f"{step}", command=lambda m=motor_letter, s=step: send_jog_command(m, s))
        jog_button.grid(row=motor-1, column=(step+1000)//10, padx=5, pady=5)

# Send Button
send_button = ttk.Button(frame, text="Send Command", command=send_custom_command)
send_button.grid(row=0, column=4, columnspan=2, pady=(10, 0))

# Calibrate Button
send_button = ttk.Button(frame, text="Calibrate", command=send_calibration_command)
send_button.grid(row=1, column=4, columnspan=2, pady=(10, 0))

# Grip Button
send_button = ttk.Button(frame, text="Grip", command=send_grip_command)
send_button.grid(row=2, column=4, columnspan=2, pady=(10, 0))

# Checkbox for Jerk
jerk_var = tk.BooleanVar()
jerk_checkbox = ttk.Checkbutton(frame, text="Jerk", variable=jerk_var)
jerk_checkbox.grid(row=3, column=4, padx=5, pady=5)

# Button to save a screenshot
screenshot_button = ttk.Button(frame, text="Take Screenshot", command=save_screenshot)
screenshot_button.grid(row=6, column=2, columnspan=2, pady=(10, 0))

# Button to save a screenshot with labelling
ssWithLabel_button = ttk.Button(frame, text="Image + Label", command=image_label)
ssWithLabel_button.grid(row=6, column=5, columnspan = 2, pady=(10,0))
# Button to delete ss + label
delete_button = ttk.Button(frame, text="Delete Last", command=delete_last)
delete_button.grid(row=7, column=5, columnspan=2, pady=(10,0))

# Button to retrieve chessboard corners
chessboard_button = ttk.Button(frame, text="Detect Chessboard", command=find_chessboard)
chessboard_button.grid(row = 7, column=2, columnspan=2, pady=(10,0))

# Create a frame for the camera view
camera_frame = ttk.Frame(root, padding="20")
camera_frame.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N, tk.S), rowspan=5)

# Placeholder label for the camera view
camera_label = ttk.Label(camera_frame, text="Camera View")
camera_label.pack(expand=True, fill=tk.BOTH)

# Start updating the camera view
update_camera_view()

# ... (existing code)
#send_serial_command("calibrate")
#send_serial_command("J190J2125J3-90J40J5-100J60")

ser.read
root.mainloop()

# Release the camera and close the serial port when the GUI is closed
videout.release()
camera.release()
ser.close()