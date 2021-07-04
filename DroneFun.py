from djitellopy import tello
from time import sleep, time
import cv2
import pygame

#region drone initialization
drone = tello.Tello()
drone.connect()
print(f'average temperature of drone: {drone.get_temperature()}')
print(f'current drone battery level: {drone.get_battery()}%')
global img
# drone.takeoff()
#endregion

#region basic movements
# drone.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
# drone.send_rc_control(0,50,0,0)
# sleep(2)
# drone.send_rc_control(0,0,0,30)
# sleep(2)
# drone.send_rc_control(0,0,0,0)
# drone.land()
#endregion

#region image capture
# drone.stream_on()

# while True:
#     img = drone.get_frame_read().frame
#     img = cv2.resize(img, (360,240))
#     cv2.imshow('Image', img)
#     cv2.waitKey(1)

#endregion

#region keyboard controls
# def init():
#     pygame.init()
#     win = pygame.display.set_mode((400,400))
# init()

# # acquiring key strokes
# # for more details, check out https://www.geeksforgeeks.org/how-to-get-keyboard-input-in-pygame/
# def getKey(keyCode:str) -> bool:
#     ans = False
#     for event in pygame.event.get(): pass
#     keyPressed = pygame.key.get_pressed()
#     myKey = getattr(pygame, f'K_{keyCode}')
#     if keyPressed[myKey]:
#         ans = True
#     pygame.display.update()
#     return ans

# def getCustomizedKeyboardInputs(lrSpeed=0, fbSpeed=0, udSpeed=0, yvSpeed=0) -> list[int]:
#     lr,fb,ud,yv = 0,0,0,0
#     if getKey('LEFT'): lr = -lrSpeed
#     elif getKey('RIGHT'): lr = lrSpeed
    
#     if getKey('UP'): fb = -fbSpeed
#     elif getKey('DOWN'): fb = fbSpeed
    
#     if getKey('w'): ud = -udSpeed
#     elif getKey('s'): ud = udSpeed
    
#     if getKey('a'): yv = -yvSpeed
#     elif getKey('d'): yv = yvSpeed
    
#     # take off if on the ground
#     if drone.get_height() < 2:
#         if getKey('t'): drone.takeoff()
    
#     # quit flying
#     if getKey('q'): drone.land(); sleep(3)
    
#     if getKey('z'):
#         cv2.imwrite(f'Resources/Images/{time()}.jpg', img)
#         sleep(0.5)
    
#     return [lr,fb,ud,yv]

#endregion

#region surveillance
# control the drone
# using this coroutine
# while True:
#     lr,fb,ud,yv = getCustomizedKeyboardInputs(50,50,50,50)
#     drone.send_rc_control(lr,fb,ud,yv)
    
#     # image capturing
#     img = drone.get_frame_read().frame
#     img = cv2.resize(img, (360,240))
#     cv2.imshow('Image', img)
#     cv2.waitKey(1)


#endregion


