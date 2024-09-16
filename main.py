import image_processor
import camera
import motion
import cv2
import time
import math

def main_loop():
    debug = True  
    robot_motion = motion.OmniMotionRobot(port="/dev/ttyACM0")
    cam = camera.RealsenseCamera(exposure=100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    robot_motion.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    center_x = cam.rgb_width // 2  # Center of the frame width
    buffer = 50  # Tolerance for ball to be considered "centered"
    max_angular_speed = 100  # Maximum angular velocity (curving)
    max_linear_speed = 60  # Maximum linear speed (forward)
    searching_for_ball = True  # Flag to indicate if the robot is searching for the ball
    ball_picked_up = False  # Flag to indicate if the ball has been picked up
    thrower_motor_speed = 1000  # Speed for the thrower motor

    ball_pickup_threshold = 50

    try:
        while True:
            processed_data = processor.process_frame(aligned_depth=False)
            frame_cnt += 1

            if processed_data.balls:
                ball = processed_data.balls[-1]  # Use the last detected ball
                offset_x = ball.x - center_x 

                if ball.exists and not ball_picked_up:
                    searching_for_ball = False  # Ball found, stop searching

                    # Proportional control for angular velocity based on the offset
                    angular_velocity = (offset_x / center_x) * max_angular_speed

                    # Quadratic deceleration: slows down quickly as the ball approaches the center
                    linear_speed = max(0.05 * max_linear_speed, max_linear_speed * (1 - (abs(offset_x) / center_x) ** 2))

                    if abs(offset_x) < buffer:
                        angular_velocity = 0

                    if ball.distance <= ball_pickup_threshold:
                        robot_motion.move(0, 0, 0)

                        print("Picking up the ball with the thrower motor.")
                        robot_motion.send_command(0, 0, 0, thrower_motor_speed, 1500, 1500)
                        ball_picked_up = True

                        # After picking up the ball, stop the thrower motor
                        robot_motion.send_command(0, 0, 0, 0, 1500, 1500)  # Turn off the thrower motor
                    else:
                        # Move the robot with the calculated linear and angular velocities (smooth curving)
                        robot_motion.move(0, linear_speed, -angular_velocity)
                else:

                    searching_for_ball = True
            else:

                searching_for_ball = True

            if searching_for_ball:
                robot_motion.move(0, 0, max_angular_speed // 2)

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processed_data.balls)))

            if debug:
                debug_frame = processed_data.debug_frame
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