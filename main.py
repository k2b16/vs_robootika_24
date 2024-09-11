import image_processor
import camera
import motion
import cv2
import time

def main_loop():
    debug = True  
    robot_motion = motion.OmniMotionRobot(port="/dev/ttyACM0")
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    robot_motion.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    center_x = cam.rgb_width // 2
    buffer = 28
    searching_for_ball = True
    try:
        #time.sleep(2)
        
        #robot_motion.move(0, 0, 0, -100)
        #time.sleep(2)
        
        #robot_motion.move(0, 0, 0, 0)

        while True:
            #robot_motion.move(0, 0, 0, 100)
            processedData = processor.process_frame(aligned_depth=False)

            frame_cnt +=1

            if processedData.balls:
                ball = processedData.balls[0]

                if ball.exists:
                    searching_for_ball = False 

                    # Center the camera on the ball
                    if not (center_x - buffer <= ball.x <= center_x + buffer):
                        if ball.x < center_x - buffer:  # Ball is left of center
                            robot_motion.move(0, 0, 0, 50)  # Rotate right
                        elif ball.x > center_x + buffer:  # Ball is right of center
                            robot_motion.move(0, 0, 0, -50)  # Rotate left
                    else:
                        robot_motion.move(0, 0, 0, 0)
                else:
                    searching_for_ball = True
            else:
                searching_for_ball = True
            if searching_for_ball:
                print("Searching...")
                robot_motion.move(0, 0, 0, 10)

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        robot_motion.close()

main_loop()
