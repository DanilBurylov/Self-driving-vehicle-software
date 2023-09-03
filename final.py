import time
import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time
import RPi.GPIO as IO
IO.setwarnings(False)           #отключаем показ любых предупреждений
IO.setmode (IO.BCM)             # мы будем программировать контакты GPIO по их функциональным номерам (BCM), то есть мы будем обращаться к PIN39 как ‘GPIO19’
IO.setup(19,IO.OUT)             # инициализируем GPIO19 в качестве цифрового выхода
IO.setup(26,IO.IN)

robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

message = ""
fps = 0
fps1 = 0
fps_time = 0

port = serial.Serial('/dev/ttyS0', baudrate=115200, stopbits=serial.STOPBITS_ONE)
inn=""
speed = 0
serv = 0
rgb = 2
d1 = 100
d2 = 100
Eold = 0
Kd = 0.1
Kp = 0.1
color = 'green'
trait = 'None'
lap = 0
time_speed = 0
time_lap = time.time()
time_dat = time.time()
time_stop = 0
time_lap1 = time.time()
time_znak = time.time()
timer_p = [0,0,0,0]
p_i =0

d1_old =0
d2_old = 0
timerd1 = 0
timerd2 = 0
y_d1 = 260
y_d2 = 260

x_zn = 0
w_zn = 0
h_zn = 0
y_zn = 0
Eold_zn = 0

colour = 0
colour1 = 0

low_black=np.array([0,0,0])
up_black=np.array([100,230,50])

low_green=np.array([75,111,100])
up_green=np.array([85,255,255])

low_blue=np.array([90,90,70])
up_blue=np.array([130,255,155])

low_orange=np.array([10,110,50])
up_orange=np.array([15,255,255])

low_red=np.array([0,111,50])
up_red=np.array([7,255,255])

low_blue_line=np.array([90,0,0])
up_blue_line=np.array([120,255,255])

time_state = time.time()
state = 0
flag_povorot = 0

def pd():
    Kd = 0.1
    Kp = 0.1
    global Eold, d1, d2
    E = d2 + 20 - d1
    Up = E*Kp
    Ud = (E-Eold)*Kd
    Eold = E
    U = Up + Ud
    return U

def pdznak():
    global Eold_zn, x_zn, w_zn, h_zn, y_zn, colour
    Kd = 0.2
    Kp = 0.2
    U = 0
    if colour == 1:
        E = (260 - (y_zn+h_zn)*1.2)-(x_zn+w_zn)
        Up = E * Kp
        Ud = (E - Eold_zn) * Kd
        Eold_zn = E
        U = Up + Ud
    else:
        E = (220 + (y_zn + h_zn) * 1.2) - x_zn
        Up = E * Kp
        Ud = (E - Eold_zn) * Kd
        Eold_zn = E
        U = Up + Ud

    return U

def znak (frame) :
    global x_zn, w_zn, h_zn, y_zn, colour, time_znak, colour1
    x1 = 80
    x2 = 560
    y1 = 200
    y2 = 400
    dat = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)


    mask = cv2.inRange(hsv, low_red, up_red)
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    xd, yd, wd, hd = 0, 0, 0, 0
    for i in contours:
        x, y, w, h = cv2.boundingRect(i)
        if h*w > 400 and h*w > hd*wd :
            xd, yd, wd, hd = x, y, w, h



    mask = cv2.inRange(hsv, low_green, up_green)
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    xm, ym, wm, hm = 0, 0, 0, 0
    for i in contours:
        x, y, w, h = cv2.boundingRect(i)
        if h * w > 400 and h * w > hd * wd:
            xm, ym, wm, hm = x, y, w, h
    cv2.rectangle(dat, (xm, ym), (xm + wm, ym + hm), (0, 255, 255), 2)
    cv2.rectangle(dat, (xd, yd), (xd + wd, yd + hd), (0, 255, 255), 2)

    if hd>0 or hm>0:
        time_znak = time.time()
        if yd+hd > ym+hm:
            x_zn, y_zn, w_zn, h_zn = xd, yd, wd, hd
            colour = 1
            colour1 = 1
        else:
            colour = 2
            colour1 = 2
            x_zn, y_zn, w_zn, h_zn = xm, ym, wm, hm
    else:
        if time_znak + 0.1 < time.time():
            colour = 0
            x_zn, y_zn, w_zn, h_zn = 0, 0, 0, 0

def line (frame) :
    global d1, d2, d1_old, d2_old, timerd1, timerd2, y_d1, y_d2
    x1 = 0
    y1 = 260
    x2 = 200
    y2 = 300

    dat = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low_black, up_black)
    mask1 = cv2.inRange(hsv, low_blue_line, up_blue_line)
    mask1 = cv2.bitwise_not(mask1)
    mask = cv2.bitwise_and(mask,mask1)
    imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    xd1, yd1, wd1, hd1 = 0, 0, 0, 0
    d1=0
    for i in contours:
        x, y, w, h = cv2.boundingRect(i)
        a = cv2.contourArea(i)
        if a > 200 and x+w > xd1 + wd1:
            xd1, yd1, wd1, hd1 = x, y, w, h
    d1 = xd1 + wd1
    if d1 == 0:
        if timerd1 + 0.05 > time.time():
            d1 = d1_old
    else:
        d1_old = d1
        timerd1 = time.time()
    cv2.rectangle(dat, (xd1, yd1), (xd1 + wd1, yd1 + hd1), (0, 255, 255), 2)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    x1 = 440
    y1 = 260
    x2 = 640
    y2 = 300
    dat1 = frame[y1:y2, x1:x2]
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv1, low_black, up_black)
    mask1_2 = cv2.inRange(hsv1, low_blue_line, up_blue_line)
    mask1_2 = cv2.bitwise_not(mask1_2)
    mask1 = cv2.bitwise_and(mask1, mask1_2)
    imd, contours, hod = cv2.findContours(mask1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    xd2, yd2, wd2, hd2 = 200, 0, 0, 0
    d2 = 0
    for i in contours:
        x, y, w, h = cv2.boundingRect(i)
        a = cv2.contourArea(i)
        if a > 200 and 200 - x > 200 - xd2:
            xd2, yd2, wd2, hd2 = x, y, w, h
    d2 = 200 - xd2
    if d2 == 0:
        if timerd2 + 0.05 > time.time():
            d2 = d2_old
    else:
        d2_old = d2
        timerd2 = time.time()

    cv2.rectangle(dat1, (xd2, yd2), (xd2 + wd2, yd2 + hd2), (0, 255, 255), 2)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)


def povorot(frame):
    global lap, time_lap, trait, rgb, timer_p, p_i, y_d2, y_d1
    x1 = 300
    y1 = 370
    x2 = 340
    y2 = 400
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
    dat = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(dat, cv2.COLOR_BGR2HSV)

    if trait == 'None':
        xm, ym, wm, hm = 0, 0, 0, 0
        mask = cv2.inRange(hsv, low_blue, up_blue)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for i in contours:
            x, y, w, h = cv2.boundingRect(i)
            if h * w > 50 and h * w > hm * wm:
                xm, ym, wm, hm = x, y, w, h
        cv2.rectangle(dat, (xm, ym), (xm + wm, ym + hm), (0, 255, 255), 2)

        xo, yo, wo, ho = 0, 0, 0, 0
        mask = cv2.inRange(hsv, low_orange, up_orange)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for i in contours:
            x, y, w, h = cv2.boundingRect(i)
            if h * w > 50 and h * w > ho * wo:
                xo, yo, wo, ho = x, y, w, h
        cv2.rectangle(dat, (xo, yo), (xo + wo, yo + ho), (0, 255, 255), 2)



        if (hm > 0 or ho > 0 )  and time_lap + 0.5 <= time.time():
            timer_p[0] = time.time()-time_lap
            p_i = p_i + 1
            time_lap = time.time()
            lap = lap + 1

            if hm > 0 :
                rgb = 1
                trait = 'blue'
                y_d1 = 280
            if ho > 0 :
                rgb = 4
                trait = 'orange'
                y_d2 = 280


    elif trait == "blue":
        xm, ym, wm, hm = 0, 0, 0, 0
        mask = cv2.inRange(hsv, low_blue, up_blue)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for i in contours:
            x, y, w, h = cv2.boundingRect(i)
            if h * w > 50 and h * w > hm * wm:
                xm, ym, wm, hm = x, y, w, h
        cv2.rectangle(dat, (xm, ym), (xm + wm, ym + hm), (0, 255, 255), 2)


        if hm > 0 and time_lap + 0.5 < time.time():
            timer_p[p_i] = time.time() - time_lap
            p_i = p_i + 1
            if p_i == 4:
                p_i = 0
            time_lap = time.time()
            lap = lap + 1
    else:
        xo, yo, wo, ho = 0, 0, 0, 0
        mask = cv2.inRange(hsv, low_orange, up_orange)
        imd, contours, hod = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for i in contours:
            x, y, w, h = cv2.boundingRect(i)
            if h * w > 50 and h * w > ho * wo:
                xo, yo, wo, ho = x, y, w, h
        cv2.rectangle(dat, (xo, yo), (xo + wo, yo + ho), (0, 255, 255), 2)

        if ho > 0 and time_lap + 0.5 < time.time():
            timer_p[p_i] = time.time() - time_lap
            p_i = p_i + 1
            if p_i == 4:
                p_i = 0
            time_lap = time.time()
            lap = lap + 1


colour_old = 0
t_zzz = time.time()


while True:
    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0
    key = robot.get_key()

    frame = robot.get_frame(wait_new_frame=1)
    znak(frame)
    line(frame)
    povorot(frame)

    serv = 0
    if speed > 0:
        if h_zn > 0:
            if y_zn+h_zn > 180:
                colour_old = colour
                t_zzz = time.time()
            time_dat = time.time()
            serv = int(pdznak())
            if serv>35:
                servo = 35
            if serv<-35:
                servo = -35
        else:
            if trait == 'orange' and colour1 == 2:
                if time_dat + 0.5 > time.time():
                    d2 = 0

            if trait == 'blue' and colour1 == 1:
                if time_dat + 0.5 > time.time():
                    d1 = 0
            serv = int(pd())
            if trait == 'orange' and d2 == 0:
                serv = -35
            if trait == 'blue' and d1 == 0:
                serv = 35
            if d1 == 0 and d2 == 0:
                if trait == 'blue':
                    servo = 35
                if trait == 'orange':
                    servo = -35


    # if trait == 'orange':
    #     if lap > 7 and flag_povorot == 0 and colour_old == 1:
    #
    #         if state == 0:
    #
    #             speed = 0
    #             state = 1
    #             time_state = time.time()
    #             # if time.time() - t_zzz > 0.6:
    #
    #         elif state == 1:
    #             serv = 40
    #             speed = 20
    #             if time_state + 1.2 < time.time():
    #                 state = 2
    #                 time_state = time.time()
    #         elif state == 2:
    #             serv = -40
    #             speed = -20
    #             if time_state + 1.1 < time.time():
    #                 state = 3
    #                 time_state = time.time()
    #         elif state == 3:
    #             serv = 0
    #             speed = 15
    #             if time_state + 2 < time.time():
    #                 state = 4
    #                 time_state = time.time()
    #         elif state == 4:
    #             speed = 25
    #             lap = 9
    #             flag_povorot = 1
    #             trait = 'blue'
    #
    # if trait == 'blue':
    #     if lap > 7 and flag_povorot == 0 and colour_old == 1:
    #         if state == 0:
    #             speed = 0
    #             state = 1
    #             time_state = time.time()
    #         elif state == 1:
    #             serv = -40
    #             speed = 20
    #             if time_state + 1.5 < time.time():
    #                 state = 2
    #                 time_state = time.time()
    #         elif state == 2:
    #             serv = 40
    #             speed = -20
    #             if time_state + 1.2 < time.time():
    #                 state = 3
    #                 time_state = time.time()
    #         elif state == 3:
    #             serv = 0
    #             speed = 15
    #             if time_state + 2 < time.time():
    #                 state = 4
    #                 time_state = time.time()
    #         elif state == 4:
    #             speed = 25
    #             lap = 9
    #             flag_povorot = 1
    #             trait = 'orange'

    # speed=0
    # serv=20
    message = str(speed +200) + str(serv + 200) + str(rgb) + "$"
    # message = "2002004$"
    port.write(message.encode("utf-8"))


    # if port.in_waiting > 0:
    #     inn = ""
    #     t = time.time()
    #     while 1:
    #         a = str(port.read(), "utf-8")
    #         if a != '$':
    #             inn+= a
    #         else:
    #             break
    #         if t + 0.02 < time.time():
    #             break
    #     port.reset_input_buffer()
    inn = "1"
    if (IO.input(26) == False):
        time.sleep(1)
        inn = "0"

    if inn == "0" and time_speed + 1 < time.time():
        time_speed = time.time()
        if speed == 0:
            speed = 25
        else:
            speed = 0

    if lap >= 12:
        if time_lap1 + timer_p[0]*0.6 < time.time():
            speed = 0
    else:
        time_lap1 = time.time()




    cv2.rectangle(frame, (0, 0), (640, 60), (0, 0, 0), -1)
    robot.text_to_frame(frame, inn, 20, 40)
    robot.text_to_frame(frame, flag_povorot, 20, 60)

    robot.text_to_frame(frame, str(speed), 50, 40)
    robot.text_to_frame(frame, message, 20, 20)
    robot.text_to_frame(frame, 'fps=' + str(fps), 550, 20)
    robot.text_to_frame(frame, 'd1=' + str(d1), 160, 20)
    robot.text_to_frame(frame, 'd2=' + str(d2), 160, 40)
    robot.text_to_frame(frame, 'color=' + str(color), 275, 20)
    robot.text_to_frame(frame, 'trait=' + str(trait), 275, 40)
    robot.text_to_frame(frame, 'lap=' + str(lap), 430, 20)
    robot.text_to_frame(frame, 'serv=' + str(serv), 430, 40)
    robot.text_to_frame(frame, 'state=' + str(state), 550, 40)
    x1 = 80
    x2 = 560
    y1 = 220
    y2 = 400
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.rectangle(frame, (200, 400), (440, 480), (0, 0, 0), -1)
    robot.text_to_frame(frame, '0=' + str(round(timer_p[0], 1)), 210, 420)
    robot.text_to_frame(frame, '1=' + str(round(timer_p[1], 1)), 210, 440)
    robot.text_to_frame(frame, '2=' + str(round(timer_p[2], 1)), 320, 420)
    robot.text_to_frame(frame, '3=' + str(round(timer_p[3], 1)), 320, 440)
    robot.set_frame(frame, 40)
