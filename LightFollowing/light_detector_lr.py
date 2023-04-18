import sensor, image, time, math, os, pyb, socket, network, omv
from mqtt import MQTTClient

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

#omv.disable_fb(True)

SSID='AIRLab-BigLab' # Network SSID
KEY='Airlabrocks2022'  # Network key

# Init wlan module and connect to network
print("Trying to connect... (may take a while)...")

wlan = network.WINC()
wlan.active(True)
wlan.connect(SSID, key=KEY, security=wlan.WPA_PSK)

# We should have a valid IP now via DHCP
print(wlan.ifconfig())

blue_led.on()
green_led.on()
time.sleep_ms(500)
blue_led.off()
green_led.off()
time.sleep_ms(500)

#client = MQTTClient("light", "test.mosquitto.org", port=1883)

#sock = socket.socket()
#addr = socket.getaddrinfo("test.mosquitto.org", 1883)[0][-1]

#client.connect()

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
    sensor.skip_frames(time = 1000)


if __name__ == "__main__":
    clock = time.clock() # 追踪帧率
    init_sensor(pixformat=sensor.GRAYSCALE,
                framesize=sensor.HQVGA, #240x160
                gain=-50,
                autoexposure=False,
                autowhitebal=False,
                contrast=-3,
                saturation=0)

    n_cols, n_rows = 20, 40

    while(True):
        l = 0
        r = 0
        n_pixels = 0
        clock.tick()
        img = sensor.snapshot()
        for j in range(n_rows):
            for i in range(n_cols):
                l += img.get_pixel(i, j)
                r += img.get_pixel(240 - n_cols + i , j)
                n_pixels += 1


        l /= 255 * n_pixels
        r /= 255 * n_pixels
        w = r - l
        m = (r + l) / 2
        msg = "w=" + str(w) + " m=" + str(m)
        #print((l-r)/38400)
        print(msg)
        #client.publish("light", msg)
