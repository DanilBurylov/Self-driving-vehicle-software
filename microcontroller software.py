from machine import UART
uart = UART(4, 115200, stop=1)
from pyb import Pin, delay, Timer
p_gr = Pin('Y5', Pin.OUT_PP)
p_bl = Pin('Y6', Pin.OUT_PP)
p_r = Pin('Y7', Pin.OUT_PP)
p_in = Pin('Y4', Pin.IN, Pin.PULL_UP)
servo1 = pyb.Servo(3)
p = Pin('Y10')
tim = Timer(2, freq=10000)
motor = tim.channel(4, Timer.PWM, pin=p)
n1 = Pin('Y11', Pin.OUT_PP)
n2= Pin('Y12', Pin.OUT_PP)
n1.low()
n2.high()






def rgb(c):
    if c == 0:  #потушить
        p_bl.high()
        p_gr.high()
        p_r.high()
    if c == 1:  #синий
        p_bl.low()
        p_gr.high()
        p_r.high()
    if c == 2:   #зеленый
        p_bl.high()
        p_gr.low()
        p_r.high()
    if c == 3: #красный
        p_bl.high()
        p_gr.high()
        p_r.low()
    if c == 4:   #оранжевый
        p_bl.high()
        p_gr.low()
        p_r.low()


inn =""
while True:
    if uart.any():
        a = chr(uart.readchar())
        if a != "$":
            inn += a
            if len(inn) > 10:
                inn = ""
        else:
            try:
                if len(inn) == 7:
                    speed = int(inn[0:3])-200
                    serv = int(inn[3:6])-200
                    rgb1 = int(inn[6:7])
                    #print(speed,serv,rgb)
                    rgb(rgb1)
                    servo1.angle(serv)
                    motor.pulse_width_percent(speed)

                   

            except ValueError:
                print("err")
            inn = ""
            button = str(p_in.value())
            uart.write(button + '$')





