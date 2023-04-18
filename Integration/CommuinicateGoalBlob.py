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
lab_thresh_tool = (0, 100, -63, 5, -128, 127)
lab_thresh_hand = (45,80,0,75,10,75) #(45, 95, 20, 65, 20, 65)#best so far - (45, 95, 10, 65, 20, 65)
#with auto (45, 95, 0, 65, 20, 65)
#negate = (30,61, -45,-5, -30,10)#
yellow_thresh_hand = (63, 87, -20,0,20, 60)

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

def difference_goal_rect(clock, time_start, led_pin, tracked_rect = None, night = False, show = True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    omv.disable_fb(True)
    if night:
        led_pin.value(1)
        img = sensor.snapshot().negate()
        img.lens_corr(1.65)
        img.mean(1)

        rects = img.find_rects(threshold = 8100)#70100
    else:
        elapsed = 20000 - (int((time.time_ns()-time_start)/1000))
        if elapsed > 0:
            time.sleep_us(elapsed)
        extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        extra_fb.replace(sensor.snapshot())

        led_pin.value(0)
        time.sleep(0.02)
        img = sensor.snapshot() # Take a picture and return the image.
        time_start = time.time_ns()

        img.difference(extra_fb)
        sensor.dealloc_extra_fb()
        led_pin.value(1)
        img.lens_corr(1.65)

        img.negate()#.binary( [lab_thresh_hand]).dilate(2)#.erode(1).dilate(2).erode(2).negate()#.dilate(13).erode(13)

        rects = img.find_rects(threshold = 20100)#70100

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
            tracked_rect = TrackedRect(candidates[0], 2,
                                       feature_dist_threshold=300,
                                       untracked_frame_threshold=20)

            feature_vec = tracked_rect.feature_vec


    else:
        if tracked_rect.update(rects):
            feature_vec = tracked_rect.feature_vec

        else:
            tracked_rect = None


    if show:
        st = "FPS: {}".format(str(round(clock.fps(),2)))
        if feature_vec != None:
            for i in range(4):
                img.draw_circle(feature_vec[2*i], feature_vec[2*i+1], 3, color = (0, 255, 0))
        img.draw_string(0, 0, st, color = (255,0,0)) # Note: Your OpenMV Cam runs about half as fast while
        # connected to your computer. The FPS should increase once disconnected.
        omv.disable_fb(False)
        img.flush()

    return time_start, tracked_rect


def blob_detection(clock, show = True):
    clock.tick()
    omv.disable_fb(True)
    time.sleep(0.01)
    img = sensor.snapshot().mean(1).mean(1)
    blobs = img.find_blobs([pink_thresholds],  merge=False, area_threshold=0, pixels_threshold=10, margin = 0)#margin = 30
    #print(len(blobs))
    maxBlobArea = 0
    tempBlob = None
    isActive = False
    if len(blobs) > 0:
        for blob in blobs: # find largest blob
            #x,y,w,h = blob.rect()
            #temp = [clamp(x-int(w*box_increase/2),0, img.width()), clamp(y-int(h*box_increase/2),0, img.height()),
            #    clamp(int(w*(1+box_increase)),0,img.width()),clamp(int(h*(1+box_increase)),0,img.height())]
            #print(blob.rect())
            #img.to_grayscale(roi = temp)
            #rects = img.find_rects(roi = temp)#, threshold = 8100)
            #print(len(rects))
            img.draw_rectangle(blob.rect(), color=(0,0,255))
    if show:
        st = "FPS: {}".format(str(round(clock.fps(),2)))

        img.draw_string(0, 0, st, color = (255,0,0)) # Note: Your OpenMV Cam runs about half as fast while
        # connected to your computer. The FPS should increase once disconnected.
        omv.disable_fb(False)
        img.flush()

    return 0,0,0,0

def sensor_init():

    sensor.reset() # Initialize the camera sensor.
    sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
    sensor.set_framesize(sensor.QQVGA) # or sensor.QQVGA (or others) HQVGA
    sensor.set_contrast(-3)
    sensor.set_brightness(3)
    sensor.set_saturation(2)
    sensor.set_auto_whitebal(False) # Turn off white balance.
    sensor.set_auto_gain(False, gain_db=18)
    sensor.set_auto_exposure(False, exposure_us=500)#MAKE SURE BATTER IS FULL BEFORE TUNING
    sensor.__write_reg(0x0E, 0b00000000)
    sensor.__write_reg(0x3E, 0b00000000)
    #sensor.__write_reg(0x01, 0b11000000)
    #sensor.__write_reg(0x02, 0b01111000)
    #sensor.__write_reg(0x03, 0b01110100)
    sensor.__write_reg(0x2D, 0b00000000)
    sensor.__write_reg(0x2E, 0b00000000)
    sensor.__write_reg(0x35, 0b10000000)
    sensor.__write_reg(0x36, 0b10000000)
    sensor.__write_reg(0x37, 0b10000000)
    sensor.__write_reg(0x38, 0b10000000)
    sensor.__write_reg(0x39, 0b10000000)
    sensor.__write_reg(0x3A, 0b10000000)
    sensor.__write_reg(0x3B, 0b10000000)
    sensor.__write_reg(0x3C, 0b10000000)
    sensor.skip_frames(time = 500) # Let new settings take affect.
    #sensor.set_auto_exposure(False)
    clock = time.clock() # Tracks FPS.
    led_pin = Pin("P9", Pin.OUT)
    return led_pin, clock

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
    sensor.skip_frames(time = 1000)

    #sensor.__write_reg(0x0E, 0b00000000)
    #sensor.__write_reg(0x3E, 0b00000000)
    #sensor.__write_reg(0x01, 0b11000000)
    #sensor.__write_reg(0x02, 0b01111000)
    #sensor.__write_reg(0x03, 0b01111111)
    sensor.__write_reg(0x2D, 0b00000000)
    sensor.__write_reg(0x2E, 0b00000000)
    sensor.__write_reg(0x35, 0b10000000)
    sensor.__write_reg(0x36, 0b10000000)
    sensor.__write_reg(0x37, 0b10000000)
    sensor.__write_reg(0x38, 0b10000000)
    sensor.__write_reg(0x39, 0b10000000)
    sensor.__write_reg(0x3A, 0b10000000)
    sensor.__write_reg(0x3B, 0b10000000)
    sensor.__write_reg(0x3C, 0b10000000)
    sensor.skip_frames(time = 1000)

    clock = time.clock() # Tracks FPS.
    led_pin = Pin("P9", Pin.OUT)
    return led_pin, clock


def convert_data_goal(tracked_rect, data):
    if tracked_rect != None:
        data[4] = 0
        #print(tracked_rect.feature_vec)
        blx,bly,brx,bry,ulx,uly, urx,ury = tracked_rect.feature_vec
        mx = (ulx + urx + blx + brx)/4
        my = (uly + ury + bly + bry)/4
        ow = ((urx-ulx)**2 + (ury-uly)**2)**.5
        oh = ((ulx-blx)**2 + (uly-bly)**2)**.5

        #mx = (2*ox + ow) / 2
        #my = (2*oy + oh) / 2
        x = int((mx - sensor.width()/2) * 127 / sensor.width())
        y = int((my - sensor.height()/2) * 127 / sensor.height())
        w = int(ow)#int((ow) * 127 / sensor.width())
        h = int(oh)#int((oh) * 127 / sensor.height())
        data[0] = x
        data[1] = y
        data[2] = w
        data[3] = h
    else:
        if data[4] < 127:
            data[4] += 1
    return data

def send_through_uart(data, uart_port):
    msg = bytearray(8)
    msg[0] = 0x69
    msg[1] = 0x69
    msg[2] = 1
    msg[3] = data[0]
    msg[4] = data[1]
    msg[5] = data[2]
    msg[6] = data[3]
    msg[7] = data[4]
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




if __name__ == "__main__":

    networking = True
    night = False

    network_if = None
    interface = None
    if networking:
        SSID='AIRLab-BigLab' # Network SSID
        KEY='Airlabrocks2022'  # Network key
        network_if = setup_network(SSID,KEY)
        while not network_if.isconnected():
            network_if = setup_network(SSID,KEY)
        interface = setInterface(network_if)


    led_pin, clock = init_sensor(pixformat=sensor.RGB565, framesize=sensor.QQVGA, windowsize=None,
                gain=6, autoexposure=False, exposure=10000, autowhitebal=False,
                contrast=-3, saturation=2)

    time_start = time.time_ns()
    tracked_rect = None

    uart = UART(3, 115200, timeout_char=100, parity=None)
    data = [0,0,0,0,0]

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
            if (time.time_ns()-flag_last)/1000000000 > 10:
                test = time.time_ns()
                flag = getFlag(interface, network_if, flag)
                flag_last = time.time_ns()
                print("connection time: ", (flag_last - test)/1000000000)
        if flag == -1:
            break
        elif flag == 0: #blob detection
            led_pin.value(0)
            x,y,z,seen = blob_detection(clock) #this is where it is supposed to go
        elif flag == 1: #goal detection
            time_start, tracked_rect = difference_goal_rect(clock, time_start, led_pin, tracked_rect,night , show = True)
            data = convert_data_goal(tracked_rect, data)
            #print(data)
            send_through_uart(data, uart)
        elif flag == 2: #fully auto
            if balls_caught < 2:
                x,y,z,seen = blob_detection(clock)
                #balls_caught += catch_ball(x,y,z,boo)
            else:
                time_start, tracked_rect = difference_goal_rect(clock, time_start, led_pin, tracked_rect,night , show = True)
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

