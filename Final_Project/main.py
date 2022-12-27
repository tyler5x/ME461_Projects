BINARY_VIEW = True
GRAYSCALE_THRESHOLD = (240, 255)
MAG_THRESHOLD = 4
THETA_GAIN = 40.0
RHO_GAIN = 1.0
P_GAIN = 0.4 #0.7
I_GAIN = 0.7 # 0.0
I_MIN = -0.0
I_MAX = 0.0
D_GAIN = 0.1 # 0.1
STEERING_SERVO_INDEX = 0
THRESHOLD = (0, 50)
BINARY_VISIBLE = True

import sensor, image, time, math, pyb
from pyb import UART

uart = UART(3, 115200)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

# Converts the position of the line to a theta value between [-1:1] and
# rho between -40 and 40
def line_to_theta_and_rho(line):
    if line.rho() < 0:
        if line.theta() < 90:
            return (math.sin(math.radians(line.theta())),
                math.cos(math.radians(line.theta() + 180)) * -line.rho())
        else:
            return (math.sin(math.radians(line.theta() - 180)),
                math.cos(math.radians(line.theta() + 180)) * -line.rho())
    else:
        if line.theta() < 90:
            if line.theta() < 45:
                return (math.sin(math.radians(180 - line.theta())),
                    math.cos(math.radians(line.theta())) * line.rho())
            else:
                return (math.sin(math.radians(line.theta() - 180)),
                    math.cos(math.radians(line.theta())) * line.rho())
        else:
            return (math.sin(math.radians(180 - line.theta())),
                math.cos(math.radians(line.theta())) * line.rho())

def line_to_theta_and_rho_error(line, img):
    t, r = line_to_theta_and_rho(line)
    r = r - (img.width() // 2)
    return (t, r)

old_result = 0
old_time = pyb.millis()
i_output = 0
output = 0
last_vals = [0] * 10

while True:
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD]) if BINARY_VISIBLE else sensor.snapshot()

    line = img.get_regression([(255, 255)], robust=True)
    hist = img.get_statistics()
    mean = hist.mean()
    print_string = ""

    if line and (line.magnitude() >= MAG_THRESHOLD): # Make sure line exists to avoid errors
        img.draw_line(line.line(), color=127) # Draw line on image so we can tell what's going on

        t, r = line_to_theta_and_rho_error(line, img) # Get initial theta and rho values
        new_result = (t * THETA_GAIN) + (r * RHO_GAIN) # Multiply individual error by gains so we can tune
        delta_result = new_result - old_result # Find difference in error
        old_result = new_result # Keep track of K and K_1 for calculating derivative and integral of error

        new_time = pyb.millis() # Current time
        delta_time = new_time - old_time
        old_time = new_time # Keep track of old time in order to calculate error derivative

        p_output = new_result # Proportional value is just error
        i_output = max(min(i_output + new_result, I_MAX), I_MIN) # Integral adds old error to integral counter
        d_output = (delta_result * 1000) / delta_time # (New result - old result) / time step
        pid_output = (P_GAIN * p_output) + (I_GAIN * i_output) + (D_GAIN * d_output)

        output = 128 + max(min(int(pid_output), 90), -90)

        if(abs(t) > 0.6): # line angle too extreme
            uart.writechar(0)
            print("Angle too steep - turn %d" % output)
        elif (mean > 20): # too much noise
            uart.writechar(0)
            print("Noisy Image - turn %d" % output)
        else: # line is good
            uart.writechar(int(output))
            print("Line Ok","Turn:",last_vals[-1],"Theta:",round(t,2),"Rho:",round(r,2))
    else:
        uart.writechar(0)
        print("Line Lost - turn %d" % output)

    last_vals[-1] = output
    for i in range(1, len(last_vals)): #average last 10 values
        last_vals[i-1] = last_vals[i]

