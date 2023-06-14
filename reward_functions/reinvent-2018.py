import math;

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):
            if start is None:
                start = 0
            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[3.07857, 0.7234, 4.0, 0.03587],
[3.22295, 0.71246, 4.0, 0.0362],
[3.36865, 0.70402, 4.0, 0.03649],
[3.51539, 0.69762, 4.0, 0.03672],
[3.66294, 0.69287, 4.0, 0.03691],
[3.81112, 0.68942, 4.0, 0.03705],
[3.95978, 0.68698, 4.0, 0.03717],
[4.10881, 0.68536, 4.0, 0.03726],
[4.25813, 0.68454, 4.0, 0.03733],
[4.4074, 0.68487, 4.0, 0.03732],
[4.55614, 0.68678, 4.0, 0.03719],
[4.704, 0.69061, 3.79979, 0.03893],
[4.85072, 0.69669, 3.62783, 0.04048],
[4.99598, 0.70537, 3.55948, 0.04088],
[5.13949, 0.71702, 3.40611, 0.04227],
[5.28093, 0.73198, 3.24666, 0.04381],
[5.41997, 0.75056, 3.04501, 0.04607],
[5.55628, 0.77305, 2.78884, 0.04954],
[5.68961, 0.79959, 2.58723, 0.05255],
[5.81987, 0.83014, 2.3113, 0.05789],
[5.94681, 0.86481, 2.07811, 0.06332],
[6.07015, 0.90377, 1.85132, 0.06986],
[6.18943, 0.94729, 1.65392, 0.07677],
[6.30396, 0.99586, 1.44412, 0.08614],
[6.41305, 1.04984, 1.44412, 0.08429],
[6.51548, 1.10999, 1.44412, 0.08225],
[6.60983, 1.17694, 1.3, 0.08899],
[6.69419, 1.25136, 1.3, 0.08654],
[6.76624, 1.33366, 1.3, 0.08414],
[6.82221, 1.42397, 1.3, 0.08173],
[6.86523, 1.51907, 1.3, 0.08029],
[6.89274, 1.61832, 1.3, 0.07922],
[6.90063, 1.72008, 1.36025, 0.07503],
[6.89071, 1.82141, 1.42024, 0.07169],
[6.86585, 1.92062, 1.42024, 0.07201],
[6.82793, 2.01677, 1.42024, 0.07278],
[6.77364, 2.10731, 1.59281, 0.06628],
[6.70615, 2.19179, 1.75141, 0.06174],
[6.62745, 2.27016, 1.90428, 0.05832],
[6.53892, 2.34243, 2.1459, 0.05326],
[6.4423, 2.4092, 2.36971, 0.04956],
[6.33878, 2.4709, 2.63033, 0.04582],
[6.2294, 2.52805, 2.9531, 0.04179],
[6.11518, 2.58128, 3.38713, 0.0372],
[5.99717, 2.63131, 3.97861, 0.03222],
[5.87631, 2.67889, 4.0, 0.03247],
[5.75359, 2.72482, 4.0, 0.03276],
[5.62981, 2.76984, 4.0, 0.03293],
[5.49795, 2.81748, 4.0, 0.03505],
[5.36653, 2.86607, 4.0, 0.03503],
[5.23582, 2.91617, 4.0, 0.035],
[5.10609, 2.96836, 4.0, 0.03496],
[4.97753, 3.02305, 4.0, 0.03493],
[4.8503, 3.08056, 4.0, 0.0349],
[4.72449, 3.14103, 4.0, 0.0349],
[4.6001, 3.20451, 4.0, 0.03491],
[4.47711, 3.27091, 4.0, 0.03494],
[4.3554, 3.33998, 4.0, 0.03499],
[4.23479, 3.41134, 4.0, 0.03503],
[4.11504, 3.48448, 4.0, 0.03508],
[3.99593, 3.55896, 3.78405, 0.03712],
[3.87735, 3.63453, 3.44542, 0.04081],
[3.76113, 3.7075, 3.17837, 0.04318],
[3.64439, 3.77888, 2.98202, 0.04589],
[3.52693, 3.84793, 2.8372, 0.04802],
[3.40855, 3.91383, 2.73024, 0.04963],
[3.28907, 3.97573, 2.65587, 0.05067],
[3.16841, 4.03287, 2.61506, 0.05105],
[3.04654, 4.08445, 2.61506, 0.05061],
[2.92352, 4.12983, 2.61506, 0.05014],
[2.79954, 4.16845, 2.61506, 0.04966],
[2.67484, 4.19988, 2.61506, 0.04918],
[2.54972, 4.22385, 2.53202, 0.05031],
[2.42455, 4.24027, 2.40089, 0.05258],
[2.29967, 4.24929, 2.22165, 0.05636],
[2.17537, 4.25144, 2.07739, 0.05984],
[2.05199, 4.24673, 1.85209, 0.06667],
[1.92988, 4.23511, 1.62627, 0.07542],
[1.80954, 4.21626, 1.44206, 0.08447],
[1.69159, 4.1896, 1.44206, 0.08386],
[1.57699, 4.15419, 1.44206, 0.08317],
[1.46688, 4.10924, 1.44206, 0.08248],
[1.36327, 4.053, 1.44206, 0.08175],
[1.26935, 3.98333, 1.44206, 0.08109],
[1.18961, 3.89844, 1.54329, 0.07547],
[1.12366, 3.8017, 1.72915, 0.06771],
[1.06963, 3.69616, 1.88962, 0.06275],
[1.02636, 3.58353, 2.04732, 0.05894],
[0.99299, 3.46502, 2.20363, 0.05587],
[0.96886, 3.3416, 2.36054, 0.05327],
[0.95341, 3.21418, 2.51715, 0.05099],
[0.94612, 3.08358, 2.66818, 0.04902],
[0.94652, 2.95069, 2.80394, 0.0474],
[0.95419, 2.81634, 2.93246, 0.04589],
[0.96868, 2.68141, 3.01121, 0.04507],
[0.98967, 2.5467, 3.06276, 0.04452],
[1.01688, 2.41293, 3.05594, 0.04467],
[1.05003, 2.28075, 2.87043, 0.04747],
[1.0888, 2.15069, 2.68409, 0.05056],
[1.13289, 2.02315, 2.44377, 0.05522],
[1.18207, 1.89846, 2.22144, 0.06034],
[1.23616, 1.77692, 2.03357, 0.06542],
[1.29518, 1.65891, 1.756, 0.07514],
[1.35951, 1.54509, 1.756, 0.07446],
[1.42956, 1.4362, 1.756, 0.07373],
[1.50611, 1.33343, 1.756, 0.07298],
[1.59004, 1.23816, 1.756, 0.0723],
[1.68227, 1.15201, 1.756, 0.07187],
[1.785, 1.07848, 2.05431, 0.0615],
[1.89513, 1.01457, 2.27274, 0.05603],
[2.01127, 0.95888, 2.48269, 0.05188],
[2.13249, 0.91042, 2.67745, 0.04876],
[2.25813, 0.86849, 2.90626, 0.04557],
[2.38758, 0.8324, 3.09905, 0.04337],
[2.52045, 0.80169, 3.31189, 0.04118],
[2.65635, 0.77591, 3.5591, 0.03887],
[2.79492, 0.75461, 3.85658, 0.03635],
[2.93578, 0.73728, 4.0, 0.03548]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
