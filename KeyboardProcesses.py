import pygame
import math
from time import sleep

x, y = 500, 500
a = 0
yaw = 0
interval = 0.25
fSpeed = 117 / 10  # Forward Speed in cm/s   (15cm/s)
aSpeed = 360 / 10  # Angular Speed Degrees/s  (50d/s)
dInterval = fSpeed * interval
aInterval = aSpeed * interval


# region keyboard controls
# acquiring key strokes
# for more details, check out https://www.geeksforgeeks.org/how-to-get-keyboard-input-in-pygame/


def getKey(keyCode: str) -> bool:
    ans = False
    for event in pygame.event.get():
        pass
    keyPressed = pygame.key.get_pressed()
    myKey = getattr(pygame, f'K_{keyCode}')
    if keyPressed[myKey]:
        ans = True
    pygame.display.update()
    return ans
#endregion

# region get keyboard input


def getKeyboardInput(drone,exit_event, speed=80, aSpeed=100,):
    lr, fb, ud, yv = 0, 0, 0, 0
    global x, y, yaw, a
    d = 0
    if getKey('LEFT'):
        lr = -speed
        d = dInterval
        a = -180
    elif getKey('RIGHT'):
        lr = speed
        d = -dInterval
        a = 180
    if getKey('UP'):
        fb = speed
        d = dInterval
        a = 270
    elif getKey('DOWN'):
        fb = -speed
        d = -dInterval
        a = -90
    if getKey('w'):
        ud = speed
    elif getKey('s'):
        ud = -speed
    if getKey('a'):
        yv = -aSpeed
        yaw -= aInterval
    elif getKey('d'):
        yv = aSpeed
        yaw += aInterval
    if getKey('q'):
        drone.land()
        exit_event.set()
        sleep(1)
    elif getKey('t'):
        drone.takeoff()
        sleep(1)
    if getKey('z'):
        cv2.imwrite(f'Resources/Images/{time()}.jpg', img)
        # makes sure we don't save a bunch of images at once when user tries to take a snapshot
        sleep(0.5)
    sleep(interval)
    a += yaw
    x += int(d * math.cos(math.radians(a)))
    y += int(d * math.sin(math.radians(a)))
    return [lr, fb, ud, yv, x, y]
# endregion


# region getcustomizedkeyboardinputs()


def getCustomizedKeyboardInputs(drone, lrSpeed=0, fbSpeed=0, udSpeed=0, yvSpeed=0) -> list[int]:
    lr, fb, ud, yv = 0, 0, 0, 0
    if getKey('LEFT'):
        lr = - lrSpeed
    elif getKey('RIGHT'):
        lr = lrSpeed
    if getKey('UP'):
        fb = fbSpeed
    elif getKey('DOWN'):
        fb = - fbSpeed
    if getKey('w'):
        ud = udSpeed
    elif getKey('s'):
        ud = - udSpeed
    if getKey('a'):
        yv = - yvSpeed
    elif getKey('d'):
        yv = yvSpeed
    # take off if on the ground
    if drone.get_height() < 2:
        if getKey('t'):
            drone.takeoff()
    # quit flying
    if getKey('q'):
        drone.land()
        sleep(3)
    # save snapshot
    if getKey('z'):
        cv2.imwrite(f'Resources/Images/{time()}.jpg', img)
        # makes sure we don't save a bunch of images at once when user tries to take a snapshot
        sleep(0.5)
    return [lr, fb, ud, yv]
# endregion
