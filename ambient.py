import asyncio
import cv2
import numpy as np
import pyautogui
import threading
import time
import websockets


screen_width, screen_height = pyautogui.size()
compression_factor = 4
maxR = app.slider1

screenShot = []

def split_screen(screen_width, screen_height, num_areas):
    bbox_list = []
    area_width = screen_width // num_areas
    area_height = screen_height // num_areas
    for i in range(num_areas):
        for j in range(num_areas):
            bbox = (i*area_width, j*area_height, (i+1)*area_width, (j+1)*area_height)
            bbox_list = bbox_list + [bbox]
    return bbox_list


def worker(num, i, iavg_colors, prev_avg_colors, prev_prev_avg_colors):

    screen = screenShot[num[1]:num[3], num[0]:num[2]]


    # cv2.imshow(("Worker "+str(i)+" screen"), screen)
    # time.sleep(2)
    # cv2.destroyAllWindows()

    # screen = cv2.cvtColor(screen, cv2.COLOR_BGR2RGB)
    # screen = cv2.resize(screen, (int(screen.shape[1]/compression_factor), int(screen.shape[0]/compression_factor)))
    avg_color_per_row = np.average(screen, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)
    if prev_avg_colors[i] is not None and prev_prev_avg_colors[i] is not None:
        inertia_second_order = 2 * avg_color - prev_avg_colors[i] - prev_prev_avg_colors[i]
        iavg_colors[i] = inertia_second_order
    else:
        iavg_colors[i] = avg_color
    prev_prev_avg_colors[i] = prev_avg_colors[i]
    prev_avg_colors[i] = avg_color
    # print("Worker",i,tabbox[i],avg_color)

screenShot = np.array(pyautogui.screenshot(region=(0,0,screen_width,screen_height)))
screenShot = cv2.cvtColor(screenShot, cv2.COLOR_BGR2RGB)
screenShot = cv2.resize(screenShot, (int(screenShot.shape[1]/compression_factor), int(screenShot.shape[0]/compression_factor)))

old_rgbstr = []
boxes = 4
tabbox = split_screen(int(screenShot.shape[1]/compression_factor-1), int(screenShot.shape[0]/compression_factor)-1, boxes)

def make_max(xd):
    global maxR
    while maxR * xd > 255:
        maxR -= 0.001
    return int(maxR * xd)

async def send_rgb_to_esp():
    async with websockets.connect('ws://192.168.0.36:81') as websocket:
        global maxR, old_rgbstr, tabbox, boxes, screenShot

        # tabbox = split_screen(screen_width, screen_height, boxes)

        running = True
        iavg_colors = np.zeros((boxes*boxes, 3))
        prev_avg_colors = np.zeros((boxes*boxes, 3))
        prev_prev_avg_colors = np.zeros((boxes*boxes, 3))

        while running:
            try:
                screenShot = np.array(pyautogui.screenshot(region=(0,0,screen_width,screen_height)))
                screenShot = cv2.cvtColor(screenShot, cv2.COLOR_BGR2RGB)
                screenShot = cv2.resize(screenShot, (int(screenShot.shape[1]/compression_factor), int(screenShot.shape[0]/compression_factor)))

                threads = [threading.Thread(target=worker, args=(tabbox[i], i, iavg_colors, prev_avg_colors, prev_prev_avg_colors)) for i in range(boxes*boxes)]
                [t.start() for t in threads]
                [t.join() for t in threads]

                avg_color = np.average(iavg_colors, axis=0)
                avg_color = [make_max(avg_color[2]), int(make_max(avg_color[1])*0.7), int(make_max(avg_color[2])*0.6)]

                rgbstr = '#{0:06x}'.format((avg_color[0] << 20) | (avg_color[1] << 10) | avg_color[2])

                if old_rgbstr != rgbstr:
                    print(avg_color)
                    await websocket.send(rgbstr)

                old_rgbstr = rgbstr
            except ConnectionRefusedError:
                print("Connection refused - will retry in 5 seconds.")
                await asyncio.sleep(0.2)
            except Exception as e:
                print("Error:", e)
            finally:
                running = False


while True:
    tNow = time.time()
    asyncio.get_event_loop().run_until_complete(send_rgb_to_esp())
    tOld = time.time()
    if((tOld - tNow)<0.15):
        time.sleep(0.2)
    else:
        print(tOld - tNow)

