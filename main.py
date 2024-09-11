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
    buffer = 20
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
                    if not (center_x - buffer <= ball.x <= center_x + buffer):
                        if ball.x < center_x - buffer:
                            robot_motion.move(0, 0, 0, 50)
                        elif ball.x > center_x + buffer:
                            robot_motion.move(0, 0, 0, -50)
                    else:
                        robot_motion.move(0, 0, 0, 0)

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
