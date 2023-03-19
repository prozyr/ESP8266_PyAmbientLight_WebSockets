import asyncio
import cv2
import numpy as np
from PIL import ImageGrab
import websockets
import threading
# Set the screen capture dimensions and compression factor
bbox = (0, 0, 5140, 1440)  # left, top, right, bottom
bbox1 = (0, 0, int(5140/3), int(1440/2))
bbox2 = (int(5140/3), 0, int(5140*2/3), int(1440/2))
bbox3 = (int(5140*2/3), 0, 5140, int(1440/2))
bbox4 = (0, int(1440/2), int(5140/3), 1440)
bbox5 = (int(5140/3), int(1440/2), int(5140*2/3), 1440)
bbox6 = (int(5140*2/3), int(1440/2), 5140, 1440)



compression_factor = 6*6

def worker(num, i, iavg_colors):

    # Capture the screen image and compress it
    screen = np.array(ImageGrab.grab(bbox=num))
    screen = cv2.cvtColor(screen, cv2.COLOR_BGR2RGB)
    screen = cv2.resize(screen, (int(screen.shape[1]/compression_factor), int(screen.shape[0]/compression_factor)))

    # Calculate the average RGB values
    avg_color_per_row = np.average(screen, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)
    # do some processing to calculate avg_color
    # update iavg_colors with the calculated avg_color
    iavg_colors[i] = avg_color
    print("worker")
    return

# Apply compression factor to each area
bbox1_compressed = tuple(int(i/compression_factor) for i in bbox1)
bbox2_compressed = tuple(int(i/compression_factor) for i in bbox2)
bbox3_compressed = tuple(int(i/compression_factor) for i in bbox3)
bbox4_compressed = tuple(int(i/compression_factor) for i in bbox4)

print(f"Original BBox: {bbox}")
print(f"Box 1: {bbox1} Compressed: {bbox1_compressed}")
print(f"Box 2: {bbox2} Compressed: {bbox2_compressed}")
print(f"Box 3: {bbox3} Compressed: {bbox3_compressed}")
print(f"Box 4: {bbox4} Compressed: {bbox4_compressed}")

# Define the PID class
class PID:
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output
        self.last_error = 0
        self.integral = 0
        self.delay = [0] * 3

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        for num in range(0, 2, +1):
            self.delay[num] = self.delay[num-1]
        self.delay[0] = derivative
        derivative = self.delay[1]
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, self.max_output), self.min_output)
        self.last_error = error
        return output

maxR = 25.0

oldR = 0
oldG = 0
oldB = 0

Ku = 2.4
Pu = 1.2#0.02

kp = 0.6*Ku
ki = 2.0*0.6*Ku/Pu
kd = 0.6*Ku*Pu/8.0#0.125*Ku*Pu/8.0
kmax = 255
kmin = -255

iavg_colors = [0.0,0.0,0.0,0.0]

# Create PID regulators for each RGB channel
pidR = PID(kp, ki, kd, kmin, kmax)
pidG = PID(kp, ki, kd, kmin, kmax)
pidB = PID(kp, ki, kd, kmin, kmax)

async def send_rgb_to_esp():
    async with websockets.connect('ws://192.168.0.36:81') as websocket:
        global oldR
        global oldG
        global oldB
        global maxR     # brightness booster

        global bbox1
        global bbox2
        global bbox3
        global bbox4
        global bbox5
        global bbox6
        tabbox = [bbox1, bbox2, bbox3, bbox4, bbox5, bbox6]
        running = True  # Add this variable

        # Create PID regulators for each RGB channel
        global pidR
        global pidG
        global pidB
        global iavg_colors

        while running:
            try:  # Add try-except block

                if __name__ == '__main__':
                    global iavg_colors
                    iavg_colors = [None] * 6  # initialize iavg_colors to a list of 4 None values
                    threads = []
                    for i in range(6):
                        t = threading.Thread(target=worker, args=(tabbox[i], i, iavg_colors))
                        threads.append(t)
                        t.start()

                    # wait for all threads to finish
                    for t in threads:
                        t.join()
                # print(iavg_colors)
                # Sum up the list elements to get the total average color
                # Calculate the average color
                num_colors = len(iavg_colors)
                avg_color = [sum(channel)/num_colors for channel in zip(*iavg_colors)]

                print(avg_color)

                # Update the PID regulators for each RGB channel
                dt = 1/60.0
                errorR = avg_color[0] - oldR
                outputR = pidR.update(errorR, dt)
                oldR = oldR + outputR

                errorG = avg_color[1] - oldG
                outputG = pidG.update(errorG, dt)
                oldG = oldG + outputG

                errorB = avg_color[2] - oldB
                outputB = pidB.update(errorB, dt)
                oldB = oldB + outputB

                # Convert the RGB values to hex and send to ESP8266 over WebSocket
                #rgbb = hex(int(avg_color[0]) << 20 | int(avg_color[1]) << 10 | int(avg_color[2]))
                r = int(4*maxR*oldR**2 / 1023)
                g = int(4*maxR*oldG**2 / 1023)
                b = int(4*maxR*oldB**2 / 1023)

                if r > 1023 or g > 1023 > b > 1023:
                    maxR -= 0.7


                rgb = (b << 20) | (g << 10) | r
                rgbstr = '#' + format(rgb, '06x')
                # print(rgbstr)

                await websocket.send(rgbstr)
                # print the average RGB and output values
                # print("Average RGB:", avg_color[0], avg_color[1], avg_color[2], "Output R:", oldR, "Output G:", oldG, "Output B:", oldB)
            except Exception as e:
                print("Error:", e)  # Print error message
            finally:
                running = False  # Set running variable to False

while True:
    asyncio.get_event_loop().run_until_complete(send_rgb_to_esp())
