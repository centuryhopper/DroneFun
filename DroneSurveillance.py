# region surveillance
# control the drone using this coroutine


def droneSurveillance(drone):
    while True:
        lr, fb, ud, yv = getCustomizedKeyboardInputs(drone, 50, 50, 50, 50)
        drone.send_rc_control(lr, fb, ud, yv)
        # image capturing
        img = drone.get_frame_read().frame
        # img = cv2.resize(img, (360,240))
        img = cv2.resize(img, (1920, 1080))
        cv2.imshow('Drone Surveillance', img)
        cv2.waitKey(1)
# endregion
