""" driver.py
    Author:     Carson Clarke-Magrab <ctc7359@rit.edu>
    Date:       03/23/2019
    Modified:
        03/29/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -   Adapted for IDE virtual NXP Cup.
        4/24/2020 Modified by Jake and Garrett
"""
from client.car import Actuator, Sensor
from client.graph import Graph
import math

# Global variables
MPS_PER_KMH = 1000 / 3600


# driving constants
acceptable_offset_error = 0.1 # Acceptable steady state error for offset

# Set sum max and min speeds
min_speed = 85 *MPS_PER_KMH
max_speed = 400*MPS_PER_KMH
initial_speed = 100*MPS_PER_KMH
# Constants used for taking turns
track_angle_turn = 0
focus_angles = [0] # [ (-7 -6 -5 -4 -3), (-2 -1 0 1 2), (3 4 5 6 7) ]
angle_mapping = [-15, -10, -5, 0, 5, 10, 15]

class Driver:
    """ Car driving logic
        The Driver receives a set of Sensor data from the server and transmits
        a set of Actuator data in response. See car.py for more details on
        these structures.
        The drive() function is called approximately every 20ms and must return
        an Actuator object within 10ms.
        You may add new functions and modify this class as you see fit, but do not
        remove the drive() function or change the name of the class.
    """

    def __init__(self):
        """ If you need to initialize any variables, do it here and remove the
            'pass' statement.
        """

        # Define class variables here
        # speed parameters
        self.prev_speed = 0
        self.speed_prev_2 = 0
        self.prev_speed_error = 0
        self.reference_speed = 10

        # steering parameters
        self.reference_angle = 0
        self.prev_angle = 0
        self.prev_angle_error = 0
        self.prev_offset = 0
        self.prev_offset_error = 0
        self.reference_offset = 0
        self.last_distance_read = [0]*15
        self.prev_last_distance_read = [0]*15
        self.current_state = 0
        self.turning_state = 1
        self.is_shallow = 1
        self.focus = 0

        # Braking terms
        self.prev_wheel = [0]*4
        self.prev_2_wheel = [0]*4

        # Shifting Parameters
        self.rpm_max = 8500
        self.rpm_min = 4500
        self.gear_max = 6
        self.gear_change_d = 0
        self.gear_last = 0
        self.last_time = 0

        # Graph Parameters
        # self.steer_graph = Graph(
        #     labels=("Current Steering", "Distance from Center", "Angle from Track", "Speed Error", "speed"),
        #     xmin=-1, xmax=1, title='Steering', hbar=True)
        # self.cam_graph = Graph(title="Sensors", ymin=0, ymax=200)
        # self.speed_graph = Graph(title="speeds", ymin=0, ymax= 300,labels=("Current speed", "Speed error", "Reference speed"))

    def drive(self, sensor: Sensor) -> Actuator:
        """ Produces a set of Actuator commands in response to Sensor data from
            the server .
            Args:
                sensor  - A set of Sensor values received from the server
            Returns: An Actuator populated with commands to send to the server
        """

        command = Actuator()

        # get current sensor readings
        current_angle = -(sensor.angle/180) # normalize to (-1, 1) by dividing by 180 degrees
        current_speed = sensor.speed_x # Current speed of the car in meters per second
        current_offset = sensor.distance_from_center # Distance from the center of the track from -1 to 1
        current_engine_rpm = sensor.rpm # Current RPM of the engine

        # calculate error between reference (desired) and actual
        current_speed_error = self.reference_speed - current_speed 
        current_angle_error = self.reference_angle - current_angle
        current_offset_error = self.reference_offset - current_offset

        # calculate the difference between current and previous error
        current_speed_delta = (current_speed_error - self.prev_speed_error)
        current_angle_delta = (current_angle_error - self.prev_angle_error)
        current_offset_delta = (current_offset_error - self.prev_offset_error)

        # calculate average wheel velocity
        wheel_vel = sum(list(sensor.wheel_velocities))/len(sensor.wheel_velocities)

        # define array containing the current states of the car
        # The car has multiple states of interest.
        #   Current speed of the car
        #   Current angle of the car
        #   Current offset of the car
        #   Current average wheel velocities
        #   Current Engine RPM 
        #   Absolute angle of the car
        #   Absolute offset error
        #   Current Speed Error
        #   Current Angle Error
        #   Current Offset Error
        #   Current Speed Delta Error
        #   Current Angle Delta Error
        #   Current Offset Delta Error
        states = [current_speed, current_angle, current_offset, wheel_vel, current_engine_rpm, abs(current_angle),
        abs(current_offset_error), current_speed_error, current_angle_error, current_offset_error, current_speed_delta,
        current_angle_delta, current_offset_delta]
        # The PID controller has both tracking and stability control, the states are all fed to the PID controller

        # Store information for next iteration
        self.prev_speed = current_speed
        self.prev_angle = current_angle
        self.prev_offset = current_offset
        self.prev_speed_error = current_speed_error
        self.prev_angle_error = current_angle_error
        self.prev_offset_error = current_offset_error

        # Run commands
        # Determine our next applied inputs in the car
        (u_steering, u_accelerator, u_brake) = self.pid_control(states)

        # Are we turning or not and if so how much
        self.is_car_turning(sensor, current_offset_error, current_angle_error, current_speed_error)

        # set the speed for a straight-away
        self.reference_speed = max_speed * min((sum(sensor.distances_from_edge[7:11])/len(sensor.distances_from_edge[7:11])/150)+0.2, 1)

        # if the car is turning, update the speed to a turning speed
        if(self.current_state):
            self.updateSpeed(sensor)

        # Command the car
        command.steering = u_steering
        command.accelerator = u_accelerator
        command.brake = min(u_brake, 0.8) # Limit the brake here because the PID loop is only limiting it from 0 to 1

        # shift gears
        command.gear = self.select_gear(sensor, command)

        # self.cam_graph.add(sensor.distances_from_edge)
        # self.steer_graph.add([command.steering, sensor.distance_from_center, current_angle, current_speed_error, current_speed])
        # self.speed_graph.add([sensor.speed_x/MPS_PER_KMH, current_speed_error/MPS_PER_KMH, self.reference_speed/MPS_PER_KMH])

        # check if wheels are locking up
        self.are_wheels_locked(sensor, command) # Needs to be last function called because function can overrule the actual command

        return command

    def is_car_turning(self, sensor, offset_error, angle_error, speed_error):
        """ Controls the offset of the car when taking turns and sets states that are used to control the car's speed
            when taking turns
            Args:
                sensor  - A set of Sensor values received from the server
                offset_error - the error between the reference offset and current offset
                angle_error - the error between the reference angle and the current angle
                speed_error - the error between the reference speed and current speed
            Returns: None
        """
        self.reference_angle = 0

        # get the middle values of distances from edge array
        # this is what the car sees in front of it
        data = list(sensor.distances_from_edge)[6:12]

        # remove 'DC' offset from the data
        min_value = min(data)
        for i in range(len(data)):
            data[i] = data[i] - min_value
        max_value = max(data)

        # Using Aalborg as a test track, it was found that sharp turns at this point will
        # show a small variation between min and max value while shallow turns will have a
        # big difference
        index = data.index(max_value)
        # print(data)

        # set driver focus to the angle corresponding to the furthest distance from the car
        self.focus = angle_mapping[index]

        # reset states
        self.current_state = 0
        self.turning_state = 1
        self.is_shallow = 0
        change_in_offset = 0

        # Enter this loop if a turn is detected within 150m of the car.
        # This loop sets 'current state' which helps to control the speed of the car in turns
        if (150 > (max_value + min_value)):
            # Right hand turn
            if(0 < self.focus):
                # sharp right
                if(20 > max_value):
                    change_in_offset -= 0.02
                    self.current_state = 4
                # mild right
                else:
                    change_in_offset -= 0.01
                    self.current_state = 0.05*abs(offset_error)
                    self.is_shallow = 1
            # Left hand turn
            else:
                # sharp left
                if(20 > max_value):
                    change_in_offset += 0.02
                    self.current_state = 4
                # mild left
                else:
                    change_in_offset += 0.01
                    self.current_state = 0.05*abs(offset_error)
                    self.is_shallow = 1

            # ensure the car does not offset itself off the track and changes offset smoothly
            change_in_offset = min(0.02, change_in_offset)
            self.reference_offset += change_in_offset
            self.reference_offset = min(0.8, self.reference_offset)
            self.reference_offset = max(-0.8, self.reference_offset)

        # Enter this loop if the car is turning
        if(self.current_state):
            # take the derivative of the distances from edge to see its rate of change
            Filter = [-1, 0, 1]
            derivative = data[index]*Filter[0] + self.last_distance_read[index]*Filter[1] \
             + self.prev_last_distance_read[index]*Filter[2]
            # if its changing rapidly
            if(derivative >= 2):
                self.turning_state = 2
            # if its changing slowly
            elif(2 > abs(derivative)):
                self.turning_state = 1.5
            else:
                self.turning_state = 0.5
        # bring the car back  to center smoothly
        else:
            if self.reference_offset < -0.01:
                self.reference_offset += 0.01
            elif self.reference_offset > 0.01:
                self.reference_offset -= 0.1
            else:
                self.reference_offset = 0

        # update for the next iteration
        self.prev_last_distance_read = self.last_distance_read
        self.last_distance_read = data

        # Look forward again
        self.focus = 0

    def updateSpeed(self, sensor):
        """ Updates the reference speed of the car, but is only called if the car is taking a turn
            Args:
                sensor  - A set of Sensor values received from the server
            Returns: None
        """
        #
        distance_from_edge = sum(sensor.distances_from_edge[7:11])/len(sensor.distances_from_edge[7:11])
        # So the turning speed line is pretty cool
        # We have a DC offset of min speed for the turn, we cannot set the turning speed lower
        # We then measure the distance in front of us and scale that value
        # If we are in a sharp turn, the current state is greater than
        # one, attenuating the distance from edge term and slowing down the car
        # Then if the turn is shallow and we are have a low offset error, the current state is less
        # than 1 making the car speed up more.
        # The turning state tells the car where in the turn we are, if we are approaching a turn
        # we slow down, if we are in a turn we stay slow, if we are leaving a turn we speed up
        # So slow into turn, speed out of the turn
        turning_speed = min_speed + (1/(abs(self.current_state)+1))*(1/self.turning_state)*distance_from_edge*MPS_PER_KMH + self.is_shallow*10
        # turning_speed = max_speed - (90* abs(sensor.distance_from_center))
        
        self.reference_speed = turning_speed

    def pid_control(self, states):
        """ PID Control loop which sets the values for steering, accelerating, and braking
            Args:
                states - The states that describe the car
            Returns: None
        """

    # gains = [state gains, reference error gains, reference integral gains]
    # The state gains allow for stability control to happen. This means that the if the 
    # references are all set to 0, the car will quickly come to stop in the center
    # and parallel to the track.
    # Since the state gains are desinged to return the car back to a stable state,
    # The gains should be chosen that will always return the car back to stability
    # The next gains are tracking gains, they need to be chosen to move the car to the 
    # next referencea
    #
    #   Proportional Gain: Speeds up the response of the system, has a large effect on the
    #       overshoot
    #   Integral Gain: Used to remove the steady state error in the system,
    #       Raising this value too much will lower the stability of the system, increase 
    #       the overshoot and settling time
    #   Derivative Gain: Smooths out overshoot, also introduces a lot of noise into
    #       the system. Should be close to zero if not zero. In our case, we exclude
    #       derivative gain completely
    #
    #   states = [current_speed, current_angle, current_distance_from_center, 0 ,
    #     current_engine_rpm, current_speed_error, current_angle_error,
    #     current_distance_from_center_error, current_speed_delta,
    #     current_angle_delta, current_distance_from_center_delta];

    # The PID control is still operating off the classical princple for a PI controller but for
    # a MIMO system. The errors in each reference effects the value of each input
    # in addition to this, stability control was also introduced into the controller
    # This works on the principle of if every command goes to 0, the car should come to a stable
    # state as quickly as possible. The addition of a stability controller

        k_steering = [0, -15, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0.02]
        k_accelerator = [-1, 0, 0, 0, -0.01, -1, -10, 65, 0, 0, 0.2, 0, 0]
        k_brake = [0, 0, 0, 0, 0, 0.1, 0.01, -0.05, 0, 0.2, 0, 0, 0]

        u_steering = 0
        u_accelerator = 0
        u_brake = 0

        # Run through states and increment the control inputs based on each state
        for i in range(len(states)):
            state = states[i]
            u_steering += k_steering[i] * state
            u_accelerator += k_accelerator[i] * state
            u_brake += k_brake[i] * state

        # Using some math here to add the non-linear effect of saturation
        # Tanh will limit steering from -1 to 1
        # Tanh(0.5*(abs(u) + u) limits the input from 0 to 1
        u_steering = math.tanh(u_steering)
        u_accelerator = math.tanh(0.5*(abs(u_accelerator) + u_accelerator))
        u_brake = math.tanh(0.5*(abs(u_brake) + u_brake))

        return (u_steering, u_accelerator, u_brake)

    def select_gear(self, sensor, command):
        """ Simple gear shifting algorithm. Feel free to adjust this as you see
            fit.
            Args:
                current_gear    - The current gear the car is in
                rpm             - The current RPM of the motor
            Returns: Which gear to shift to
        """
        rpm = sensor.rpm
        gear = sensor.gear
        d_since_shift = sensor.distance_raced - self.gear_change_d

        if d_since_shift < 10:
            # Don't shift if we just did.
            # Should probably be time based, but distance is easier
            pass
        elif rpm > self.rpm_max:
            # Hitting redline, we should shift up
            self.gear_last = gear
            gear = gear + 1
            self.gear_change_d = sensor.distance_raced
        elif rpm < self.rpm_min:
            self.gear_last = gear
            gear = gear - 1
            self.gear_change_d = sensor.distance_raced

        if gear < 1:
            gear = 1
        elif gear > self.gear_max:
            gear = self.gear_max

        return gear

    def are_wheels_locked(self, sensor, command):
        """ Simple function to determine if the wheels are locking up. Lets off the breaks if they are
            Args:
                sensor  - A set of Sensor values received from the server
                command - the actuator which controls the car
            Returns: None
        """
        # get wheel velocities
        wheel_velocities = sensor.wheel_velocities

        # check if any of the wheels are rapidly slowly down
        for i in range(len(wheel_velocities)):
            derivative = wheel_velocities[i] - self.prev_2_wheel[i]
            if(-500 > derivative):
                # If the brakes are locked up, let off them
                command.brake = 0

        # update for next iteration
        self.prev_2_wheel = self.prev_wheel
        self.prev_wheel = self.prev_2_wheel