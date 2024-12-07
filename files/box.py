#!/usr/bin/env python3
import rospy
import random
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState
import time

def move_object(speed, move_distance, object_name):
    
    #print(time.time())
    # Constants
    LOOP_RATE_HZ = 20           # Frequency of the control loop in Hz
    POSITION_TOLERANCE = 0.01   # Minimum distance to consider as "reached"
    TIME_STEP_SEC = 1.0 / LOOP_RATE_HZ  # Time step for each loop iteration

    # Initialize the ROS node
    #rospy.init_node('move_object', anonymous=True)
    
    # Wait for the Gazebo services to become available
    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.wait_for_service('/gazebo/get_model_state')

    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    # Directions for movement (X, Y)
    directions = [
        (move_distance, 0),   # Move along +X-axis
        (0, move_distance),   # Move along +Y-axis
        (-move_distance, 0),  # Move along -X-axis
        (0, -move_distance)   # Move along -Y-axis
    ]

    # Get the current position of the object
    model_state = get_model_state(object_name, "world")
    current_position_x = model_state.pose.position.x
    current_position_y = model_state.pose.position.y

    # Iterate over all directions
    for direction in directions:
        t = rospy.Time.now().to_sec()
        target_position_x = current_position_x + direction[0]
        target_position_y = current_position_y + direction[1]

        rate = rospy.Rate(LOOP_RATE_HZ)

        while (
            abs(current_position_x - target_position_x) > POSITION_TOLERANCE or
            abs(current_position_y - target_position_y) > POSITION_TOLERANCE
        ):
            # Calculate the step size for each axis
            step_x = speed * TIME_STEP_SEC if current_position_x < target_position_x else -speed * TIME_STEP_SEC
            step_y = speed * TIME_STEP_SEC if current_position_y < target_position_y else -speed * TIME_STEP_SEC
            d = (direction[0] ** 2 + direction[1] ** 2) ** 0.5
            step_x = direction[0] / (d/speed) * TIME_STEP_SEC
            step_y = direction[1] / (d/speed) * TIME_STEP_SEC

            # Update current positions
            if abs(current_position_x - target_position_x) > abs(step_x):
                current_position_x += step_x
            else:
                current_position_x = target_position_x

            if abs(current_position_y - target_position_y) > abs(step_y):
                current_position_y += step_y
            else:
                current_position_y = target_position_y

            # Create and send the state message
            state_msg = ModelState()
            state_msg.model_name = object_name
            state_msg.pose.position.x = current_position_x
            state_msg.pose.position.y = current_position_y
            state_msg.pose.position.z = model_state.pose.position.z  # Keep Z constant
            state_msg.pose.orientation = model_state.pose.orientation  # Keep orientation constant
            state_msg.twist.linear.x = step_x / TIME_STEP_SEC  # Linear speed approximation
            state_msg.twist.linear.y = step_y / TIME_STEP_SEC
            state_msg.reference_frame = "world"

            resp = set_model_state(state_msg)
            if not resp.success:
                rospy.logerr(f"Failed to move object: {resp.status_message}")
                return

            rate.sleep()

        #rospy.loginfo(f"Reached target position: ({target_position_x:.2f}, {target_position_y:.2f})")

        # Update the current position for the next direction
        current_position_x = target_position_x
        current_position_y = target_position_y
        #print(rospy.Time.now().to_sec() - t)


if __name__ == '__main__':
    rospy.init_node('move_object', anonymous=True)
    try:
        # Randomize speed and define parameters
        MIN_SPEED = 0.2
        MAX_SPEED = 0.6
        MOVE_DISTANCE = 2.83
        OBJECT_NAME = "box"  # Replace with your Gazebo model name


        speed = random.uniform(MIN_SPEED, MAX_SPEED)  # Randomized speed
        rospy.loginfo(f"Randomized speed for box: {speed:.2f} m/s")
        #speed = 0.5

        while not rospy.is_shutdown():
            try:
                move_object(speed, MOVE_DISTANCE, OBJECT_NAME)
            except:
                rospy.loginfo(f"box error")
    except rospy.ROSInterruptException:
        pass
