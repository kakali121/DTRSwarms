import math
import sensor, image, time, pyb, omv
import image, network, rpc, struct, tf
from pyb import UART

# Disable built-in framebuffer.
omv.disable_fb(True)

# Set up network interface
def setup_network(ssid, key):
	network_if = network.WLAN(network.STA_IF)
	network_if.active(True)
	network_if.connect(ssid, key)
	print(network_if.ifconfig)
	interface = rpc.rpc_network_slave(network_if)
	return interface

# Set up sensor
def init_sensor(pixformat=sensor.RGB565, framesize=sensor.HQVGA, windowsize=None,
				gain=18, autoexposure=False, exposure=5000, autowhitebal=False,
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
	sensor.__write_reg(0x0E, 0b00000000)
	sensor.__write_reg(0x3E, 0b00000000)
	sensor.__write_reg(0x01, 0b11000000)
	sensor.__write_reg(0x02, 0b01111000)
	sensor.__write_reg(0x03, 0b01110100)
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
	
# Find the largest blob
def find_max(blobs):
	max_size = 0
	for blob in blobs:
		if blob.area() > max_size:
			max_blob = blob
			max_size = blob.area()
	return max_blob

# Color detection
def color_detection(data):
	img = sensor.snapshot().mean(1)
	blobs = img.find_blobs([ballon_threshold], merge=True, area_threshold=25, pixels_threshold=10)
	if blobs:
		red_led.on()
		max_blob = find_max(blobs)
		return struct.pack("<HHHH", max_blob.cx(), max_blob.cy(), max_blob.w(), max_blob.h())
	else:
		red_led.off()
		return struct.pack("<HHHH", 0, 0, 0, 0)
	

if __name__ == "__main__":
	red_led = pyb.LED(1)
	green_led = pyb.LED(2)
	blue_led = pyb.LED(3)
	red_led.off()
	green_led.off()
	blue_led.off()
	clock = time.clock()
	SSID='AIRLab-BigLab'
	KEY='Airlabrocks2022'
	interface = setup_network(SSID, KEY)
	ballon_threshold = (15, 60, 40, 90, -40, 20)
	init_sensor(pixformat=sensor.RGB565, framesize=sensor.QQVGA, gain=18, autoexposure=False,
				autowhitebal=False, exposure=10000, contrast=-3, saturation=0)
	while(True):
		interface.register_callback(color_detection)
		interface.loop()