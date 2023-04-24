# In Memory Basic Frame Differencing Example
#
# This example demonstrates using frame differencing with your OpenMV Cam. It's
# called basic frame differencing because there's no background image update.
# So, as time passes the background image may change resulting in issues.
import math
import sensor, image, time, pyb, omv, os
import image, network, rpc, struct, tf
from pyb import UART

from machine import Pin, Signal


orange_thresholds = (180,250,45,140,20,120)
pink_threshold = (15, 70, 10, 60, -20, 15)
lab_thresh_tool = (0, 100, -63, 5, -128, 127)
lab_thresh_hand = (45,80,0,75,10,75) #(45, 95, 20, 65, 20, 65)#best so far - (45, 95, 10, 65, 20, 65)
#with auto (45, 95, 0, 65, 20, 65)
#negate = (30,61, -45,-5, -30,10)#
yellow_thresh_hand = (63, 87, -20,0,20, 60)
green_thresh_hand  = (23,75,-25,0,0,20)

pink_thresholds = (30,75, 40,70,-40,20)#(30, 75, 35, 70, -40, 0)


TRIGGER_THRESHOLD = 5

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

def one_norm_dist(v1, v2):
    return sum([abs(v1[i] - v2[i]) for i in range(len(v1))])


def two_norm_dist(v1, v2):
    return math.sqrt(sum([(v1[i] - v2[i])**2 for i in range(len(v1))]))


class TrackedRect:
    def __init__(self, init_rect, norm_level: int,
                       feature_dist_threshold=300,
                       untracked_frame_threshold=20):
        self.feature_vec = []
        for i in range(4):
            for j in range(2):
                self.feature_vec.append(init_rect.corners()[i][j])
        self.norm_level = norm_level
        self.untracked_frames = 0
        self.feature_dist_threshold = feature_dist_threshold
        self.untracked_frame_threshold = untracked_frame_threshold
        #print("new rectangle")

    def update(self, rects):
        min_dist = 32767
        candidate_feature = None
        for r in rects:
            # calculate new feature vector
            bad_rect = False
            for x in range(4):
                    if r.corners()[x][0] == 0 and r.corners()[x][1] == 0:
                        bad_rect = True
            if not bad_rect:
                feature_vec = []
                for i in range(4):
                    for j in range(2):
                        feature_vec.append(r.corners()[i][j])
                if self.norm_level == 1:
                    dist = one_norm_dist(self.feature_vec, feature_vec)
                elif self.norm_level == 2:
                    dist = two_norm_dist(self.feature_vec, feature_vec)
                else:
                    # we do not need any other norm now
                    assert(False)
                if dist < min_dist:
                    min_dist = dist
                    candidate_feature = feature_vec
        if min_dist < self.feature_dist_threshold:
            self.feature_vec = candidate_feature
            self.untracked_frames = 0
            #print("replaced! {}".format(min_dist))
            # still valid
            return True
        else:
            self.untracked_frames += 1
            #print("Dropped? {}".format(min_dist))
            if self.untracked_frames >= self.untracked_frame_threshold:
                # invalid rectangle now
                return False
        return True

slow_led = False
def difference_goal_rect(clock, time_start, led_pin,extra_fb = None, tracked_rect = None, night = False, show = True, quick = False):
    clock.tick() # Track elapsed milliseconds between snapshots().
    omv.disable_fb(True)
    found_blobs = False
    if night:
        led_pin.value(1)
        img = sensor.snapshot().negate()
        img.lens_corr(1.65)
        img.mean(1)

        rects = img.find_rects(threshold = 8100)#70100
    else:
        if quick:
            elapsed = 22000 - (int((time.time_ns()-time_start)/1000))
            if elapsed > 0:
                time.sleep_us(elapsed)
            extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)

            extra_fb.replace(sensor.snapshot())
            time_start = time.time_ns()

            led_pin.value(0)

            elapsed = 21000 - (int((time.time_ns()-time_start)/1000))
            if elapsed > 0:
                time.sleep_us(elapsed)
            img = sensor.snapshot() # Take a picture and return the image.
            time_start = time.time_ns()

            img.difference(extra_fb)
            sensor.dealloc_extra_fb()
            led_pin.value(1)
            img.lens_corr(1.65)

            img.negate()#.binary( [lab_thresh_hand]).dilate(2)#.erode(1).dilate(2).erode(2).negate()#.dilate(13).erode(13)

            rects = img.find_rects(threshold = 8000)#20100)#70100
        else:
            global slow_led
            led_pin.value(slow_led)
            slow_led = not slow_led
            elapsed = 20000 - (int((time.time_ns()-time_start)/1000))
            if elapsed > 0:
                time.sleep_us(elapsed)
            time_start = time.time_ns()

            time_start = time.time_ns()
            img = sensor.snapshot() # Take a picture and return the image.
            current = img.copy()

            img.difference(extra_fb)
            extra_fb.replace(current)
            img.negate()
            rects = []



    #print(len(rects))
    feature_vec = None
    if tracked_rect == None:
        candidates = []
        for r in rects:
            rect_properties = r.rect()
            if (rect_properties[2]/rect_properties[3] >= 0.33 and
                rect_properties[2]/rect_properties[3] <= 3.0 and
                rect_properties[2]*rect_properties[3] <= 20000 and
                rect_properties[2]*rect_properties[3] >= 25):
                bad_rect = False
                for x in range(4):
                        if r.corners()[x][0] == 0 and r.corners()[x][1] == 0:
                            bad_rect = True
                if not bad_rect:
                    candidates.append(r)
        if len(candidates)>0:
            red_led.on()
            tracked_rect = TrackedRect(candidates[0], 2,
                                       feature_dist_threshold=300,
                                       untracked_frame_threshold=5)

            feature_vec = tracked_rect.feature_vec
        else:
            red_led.off()


    else:
        if tracked_rect.update(rects):
            red_led.on()
            feature_vec = tracked_rect.feature_vec

        else:
            red_led.off()
            tracked_rect = None


    if show:
        st = "FPS: {}".format(str(round(clock.fps(),2)))
        if feature_vec != None:
            for i in range(4):
                img.draw_circle(feature_vec[2*(i%4)], feature_vec[2*(i%4)+1], 3, color = (0, 255*(i%2), 255*((i+1)%2)))
        img.draw_string(0, 0, st, color = (255,0,0)) # Note: Your OpenMV Cam runs about half as fast while
        # connected to your computer. The FPS should increase once disconnected.
        omv.disable_fb(False)
        img.flush()

    return time_start, tracked_rect


# find the blob that has the most pixels among a list of blobs
def find_max(blobs):
    max_size = 0;
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob


def blob_detection(clock, show = True):
    clock.tick()
    omv.disable_fb(True)
    #time.sleep(0.01)
    img = sensor.snapshot().mean(1)

    if framesize == sensor.HQVGA:
        x_size = 240
        y_size = 160
    elif framesize == sensor.QQVGA:
        x_size = 160
        y_size = 120
    else:
        assert(False)

    blobs = img.find_blobs([pink_thresholds],
                           merge=True,
                           area_threshold=25,
                           pixels_threshold=15)
    if blobs:
        for blob in blobs:
            img.draw_rectangle(blob.rect(), color = (0,0,255))

    if blobs:
        red_led.on()
        max_blob = find_max(blobs)
        feature_vec = [max_blob.cx()/x_size, max_blob.cy()/y_size,
                       math.sqrt(x_size*y_size/(max_blob.w()*max_blob.h()))]
        dist = 0.27485909*feature_vec[2] + 0.9128014726961156
        theta = -0.98059103*feature_vec[0] + 0.5388727340530889
        phi = -0.57751757*feature_vec[1] + 0.24968235246037554
        z = dist*math.sin(phi)
        xy = dist*math.cos(phi)
        x = xy*math.cos(theta)
        y = xy*math.sin(theta)
        if show:
            img.draw_rectangle(max_blob.rect())
            st = "FPS: {}".format(str(round(clock.fps(),2)))
            img.draw_string(0, 0, st, color = (255,0,0))
            omv.disable_fb(False)
            #img.draw_string(0, 20,
            #    "x: {}\ny: {}\nz: {}".format(
            #        x, y, z), color = (255, 0, 0))
            img.flush()
        return x, y, z, True
    else:
        red_led.off()
        if show:
            st = "FPS: {}".format(str(round(clock.fps(),2)))
            img.draw_string(0, 0, st, color = (255,0,0))
            omv.disable_fb(False)
            img.flush()
        return -1, -1, -1, False


def init_sensor(pixformat=sensor.RGB565, framesize=sensor.HQVGA, windowsize=None,
                gain=18, autoexposure=False, exposure=10000, autowhitebal=False,
                contrast=0, saturation=0):
    sensor.reset()
    sensor.set_pixformat(pixformat)
    sensor.set_framesize(framesize)
    if windowsize:
        sensor.set_windowing(windowsize)

    sensor.skip_frames(time = 1000)
    sensor.set_auto_gain(False, gain_db=gain)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=exposure)
    sensor.set_contrast(contrast)
    sensor.set_saturation(saturation)
    sensor.set_brightness(3)
    sensor.skip_frames(time = 1000)
    '''
    if (sensor.get_id() == sensor.OV7725):
        sensor.__write_reg(0xAC, 0xDF)
        sensor.__write_reg(0x8F, 0xFF)'''
    #sensor.__write_reg(0x0E, 0b00000000)
    #sensor.__write_reg(0x3E, 0b00000000)
    #sensor.__write_reg(0x01, 0b01111111) # B
    #sensor.__write_reg(0x02, 0b01001000) # R
    #sensor.__write_reg(0x03, 0b01101000) # G
    #sensor.__write_reg(0x2D, 0b00000000)
    #sensor.__write_reg(0x2E, 0b00000000)
    #sensor.__write_reg(0x35, 0b10000000)
    #sensor.__write_reg(0x36, 0b10000000)
    #sensor.__write_reg(0x37, 0b10000000)
    #sensor.__write_reg(0x38, 0b10000000)
    #sensor.__write_reg(0x39, 0b10000000)
    #sensor.__write_reg(0x3A, 0b10000000)
    #sensor.__write_reg(0x3B, 0b10000000)
    #sensor.__write_reg(0x3C, 0b10000000)
    sensor.skip_frames(time = 1000)

    clock = time.clock() # Tracks FPS.
    led_pin = Pin("P9", Pin.OUT)
    return led_pin, clock

def convert_data_goal(tracked_rect, data):
    data[0] = 1
    if tracked_rect != None:
        data[5] = 0
        #print(tracked_rect.feature_vec)
        blx,bly,brx,bry,ulx,uly, urx,ury = tracked_rect.feature_vec
        mx = (ulx + urx + blx + brx)/4
        my = (uly + ury + bly + bry)/4
        u = ((urx-ulx)**2 + (ury-uly)**2)**.5
        l = ((ulx-blx)**2 + (uly-bly)**2)**.5
        b = ((brx-blx)**2 + (bry-bly)**2)**.5
        r = ((urx-brx)**2 + (ury-bry)**2)**.5
        #print(u,l,b,r)

        #mx = (2*ox + ow) / 2
        #my = (2*oy + oh) / 2
        x = int((mx - sensor.width()/2) * 127 / sensor.width())
        y = int((my - sensor.height()/2) * 127 / sensor.height())
        w = int(clamp(1/max(u,l,b,r)*1000, 0, 40)/40 * 127)
        s = int(clamp((r-l)/10,-1,1)*127)
        data[1] = x
        data[2] = y
        data[3] = int(data[3] * .9 + w * .1)
        data[4] = int(data[4] * .9 + s * .1)
        print(data[1:5])
    else:
        if data[5] < 127:
            data[5] += 1
    return data

def convert_data_blob(x,y,z,seen, data):
    data[0] = 0
    if seen:
        data[5] = 0
        data[1] = int((clamp(x, -5,5)/5)*127)
        data[2] = int((clamp(x, -5,5)/5)*127)
        data[3] = int((clamp(x, -5,5)/5)*127)
        data[4] = 0
    else:
        if data[5] < 127:
            data[5] += 1
    return data

def convert_data_light(w,m, data):
    data[0] = 1

    data[5] = 0
    data[1] = int((clamp(w, -1,1)/1)*127)
    data[2] = 0
    data[3] = int((clamp(m, 0,1)/1)*127)
    data[4] = 0


    return data

def send_through_uart(data, uart_port):
    msg = bytearray(8)
    msg[0] = 0x69
    msg[1] = 0x69
    msg[2] = data[0]
    msg[3] = data[1]
    msg[4] = data[2]
    msg[5] = data[3]
    msg[6] = data[4]
    msg[7] = data[5]
    print(data)
    uart_port.write(msg)         # send 8 byte message
    # time.sleep(0.1)         # 100ms delay

def setup_network(ssid, key):
    network_if = network.WLAN(network.STA_IF)
    network_if.active(True)
    network_if.connect(ssid, key)
    return network_if

class MyCustomException(Exception):
    pass


def throw_end(data):
    raise MyCustomException("-1")
def throw_blob(data):
    raise MyCustomException("0")
def throw_goal(data):
    raise MyCustomException("1")
def throw_auto(data):
    raise MyCustomException("2")
def loop_interrupt(data = None):
    raise MyCustomException("3")

def setInterface(network_if):
    interface = rpc.rpc_network_slave(network_if)
    interface.register_callback(throw_end)
    interface.register_callback(throw_blob)
    interface.register_callback(throw_goal)
    interface.register_callback(throw_auto)
    interface.setup_loop_callback(loop_interrupt)
    return interface


def getFlag(interface, network_if, base):

    #SSID='AIRLab-BigLab' # Network SSID
    #KEY='Airlabrocks2022'  # Network key
    #network_if = setup_network(SSID,KEY)
    #network_if = network_if.connect(SSID,KEY)
    if network_if.isconnected():
        #interface = setInterface(network_if)
        flag = -1
        try:
            interface.loop(recv_timeout=250, send_timeout=250)
        except MyCustomException as e:
            flag = int(f"{e}")
            if flag == 3:
                flag = base
            print(f"caught: {e}")
        return flag
    else:
        return base

def light_following():
    n_cols, n_rows = int(sensor.width()/2), sensor.height()
    img = sensor.snapshot()
    l = 0
    r = 0
    n_pixels = 0
    for j in range(n_rows):
        for i in range(n_cols):
            l += img.get_pixel(i, j)
            r += img.get_pixel(sensor.width() - n_cols + i , j)
            n_pixels += 1
    l /= 255 * n_pixels
    r /= 255 * n_pixels
    w = r - l
    m = (r + l) / 2
    w = 1 if w>0 else -1
    msg = "w=" + str(w) + " m=" + str(m)
    #print(msg)
    return w, m



if __name__ == "__main__":

    networking = False
    night = False
    show = True

    lightfollow = False
    network_if = None
    interface = None
    if networking:
        SSID='AIRLab-BigLab' # Network SSID
        KEY='Airlabrocks2022'  # Network key
        network_if = setup_network(SSID,KEY)
        while not network_if.isconnected():
            network_if = setup_network(SSID,KEY)

        interface = setInterface(network_if)

    framesize = sensor.QQVGA
    red_led = pyb.LED(1)
    green_led = pyb.LED(2)
    blue_led = pyb.LED(3)

    if lightfollow:
        led_pin, clock = init_sensor(pixformat=sensor.GRAYSCALE, framesize=framesize, windowsize=None,
                gain=-50, autoexposure=False, exposure=100, autowhitebal=False,
                contrast=-3, saturation=0)
    else:
        led_pin, clock = init_sensor(pixformat=sensor.RGB565, framesize=framesize, windowsize=None,
                gain=0, autoexposure=False, exposure=1000, autowhitebal=False,
                contrast=-3, saturation=0)


    quick = True
    extra_fb = None
    if not quick:
        extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        extra_fb.replace(sensor.snapshot())
        #extra_fb = sensor.snapshot()

    time_start = time.time_ns()
    tracked_rect = None

    uart = UART(3, 115200, timeout_char=50, parity=None)#115200
    data = [0,0,0,0,0,0]

    flag_last = time.time_ns()
    balls_caught = 0
    scoring = 0
    x = 0
    y = 0
    z = 0
    seen = 0
    state = 0
    print("getting started")

    flag = 1
    #flag = getFlag(interface, network_if, flag)
    while(True):
        if networking:
            if (time.time_ns() -flag_last)/1000000000 > 10:
                test = time.time_ns()
                flag = getFlag(interface, network_if, flag)
                flag_last = time.time_ns()
                print("connection time: ", (flag_last - test)/1000000000)
        if lightfollow:
            w, m = light_following()
            data = convert_data_light(w,m,data)
            send_through_uart(data, uart)

        elif flag == -1:
            break
        elif flag == 0: #blob detection
            led_pin.value(0)
            x,y,z,seen = blob_detection(clock, show = True) #this is where it is supposed to go
            data = convert_data_blob(x,y,z,seen,data)
            send_through_uart(data, uart)
        elif flag == 1: #goal detection

            time_start, tracked_rect = difference_goal_rect(clock, time_start, led_pin,extra_fb, tracked_rect,night , show, quick)
            data = convert_data_goal(tracked_rect, data)
            #print(data)
            send_through_uart(data, uart)


        elif flag == 2: #fully auto
            if balls_caught < 2:
                x,y,z,seen = blob_detection(clock, show = True)
                #balls_caught += catch_ball(x,y,z,boo)
            else:
                time_start, tracked_rect = difference_goal_rect(clock, time_start, led_pin,extra_fb, tracked_rect,night , show, quick)
                data = convert_data_goal(tracked_rect, data)
                print(data)
                #scoring = score_goal(x,y,z,boo)
            if scoring:
                state = 4
            elif balls_caught < 2:
                if seen:
                    state = 1
                else:
                    state = 0
            else:
                if seen:
                    state = 3
                else:
                    state = 2


        #print(clock.fps())

