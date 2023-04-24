import sensor, image, time, math

thresholds = [(250, 255)] # grayscale thresholds设置阈值

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.HQQVGA)
sensor.skip_frames(time = 1000)
if (sensor.get_id() == sensor.OV7725):
    sensor.__write_reg(0xAC, 0xDF)
    sensor.__write_reg(0x8F, 0xFF)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot().mean(1)
    img_ver = img.copy()
    #img_old = img.copy()
    #img.morph(2, (0, 0, 0, 0, 0,
                  #0, 0, 0, 0, 0,
                  #0, 0, 1, 1, 1,
                  #0, 0, 1, 0, 0,
                  #0, 0, 1, 0, 0))
    img_hor = img_ver.copy()

    #img.morph(2, (-1, -1, -4, -1, -1,
                  #-1, -1, -4, -1, -1,
                  #-4, -4, 12, 5, 5,
                  #-1, -1, 5, -1, -1,
                  #-1, -1, 5, -1, -1,), threshold=True)
    #img_ver.morph(1, (1, 0, -1,
                      #2, 0, -2,
                      #1, 0, -1), mul=0.1)

    #img_hor.morph(1, (1, 2, 1,
                      #0, 0, 0,
                      #-1, -2, -1), mul=0.1)
    #img_ver.morph(1, (0, 0, 0,
                      #2, 0, -2,
                      #1, 0, -1), mul=0.1)

    #img_hor.morph(1, (0, 2, 1,
                      #0, 0, 0,
                      #0, -2, -1), mul=0.1)

    img_ver.morph(2, (0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0,
                      1, 1, 0, -1, -1,
                      1, 1, 0, -1, -1,
                      1, 1, 0, -1, -1), mul=0.1)

    img_hor.morph(2, (0, 0, 1, 1, 1,
                      0, 0, 1, 1, 1,
                      0, 0, 0, 0, 0,
                      0, 0, -1, -1, -1,
                      0, 0, -1, -1, -1), mul=0.1)

    ang_eps = 0.175
    mag_eps = 50

    for u in range(0, 120, 1):
        for v in range(0, 80, 1):
            Iy = img_ver.get_pixel(u, v)
            Ix = img_hor.get_pixel(u, v)
            ang = math.atan2(Iy, Ix)
            mag = math.sqrt(Iy**2 + Ix**2)
            if math.fabs(ang - math.pi/4) <= ang_eps and mag >= mag_eps:
                img.draw_circle(u, v, 3)

    #kpts = img.find_keypoints(scale_factor=1.01)
    #if not kpts:
        #print("Huh")


    #img.binary(thresholds)
    #img.erode(1, 2)

    print(clock.fps())

