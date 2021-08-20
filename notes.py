# region basic movements
# drone.send_rc_control(left_right_velocity, forward_backward_velocity, up_down_velocity, yaw_velocity)
# drone.send_rc_control(0, 50, 0, 0)
# sleep(2)
# drone.send_rc_control(0, 0, 0, 30)
# sleep(2)
# drone.send_rc_control(0, 0, 0, 0)
# drone.land()
# endregion

# region image capture
# while True:
#     img = drone.get_frame_read().frame
#     img = cv2.resize(img, (360, 240))
#     cv2.imshow('Image', img)
#     cv2.waitKey(1)
# endregion
