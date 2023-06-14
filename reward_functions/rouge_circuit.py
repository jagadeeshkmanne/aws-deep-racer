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
        racing_track = [[-2.03192, -5.93321, 4.0, 0.07447],
[-1.73802, -5.98175, 4.0, 0.07447],
[-1.44382, -6.02623, 4.0, 0.07439],
[-1.14967, -6.06658, 3.8908, 0.07631],
[-0.85589, -6.10279, 3.51906, 0.08411],
[-0.56273, -6.1347, 3.19421, 0.09232],
[-0.2704, -6.16214, 2.88148, 0.1019],
[0.02084, -6.18467, 2.57926, 0.11325],
[0.31072, -6.20168, 2.28953, 0.12683],
[0.59899, -6.21256, 2.0231, 0.14259],
[0.88521, -6.21611, 1.74262, 0.16426],
[1.16889, -6.21098, 1.52662, 0.18586],
[1.44941, -6.19549, 1.52662, 0.18403],
[1.7259, -6.16746, 1.52662, 0.18204],
[1.9971, -6.1241, 1.52662, 0.1799],
[2.26119, -6.06188, 1.52662, 0.17772],
[2.51482, -5.97549, 1.52662, 0.17551],
[2.75259, -5.85975, 1.71512, 0.15419],
[2.976, -5.72255, 1.81062, 0.1448],
[3.18544, -5.56733, 1.91687, 0.136],
[3.38166, -5.39692, 2.01707, 0.12884],
[3.56534, -5.21345, 1.90593, 0.13622],
[3.73701, -5.01844, 1.90593, 0.13632],
[3.89711, -4.81308, 1.90593, 0.13662],
[4.04539, -4.59781, 1.90593, 0.13714],
[4.18118, -4.3728, 1.90593, 0.13789],
[4.30326, -4.13785, 1.90593, 0.13892],
[4.40733, -3.89116, 2.01206, 0.13307],
[4.49422, -3.63464, 2.47383, 0.10948],
[4.5692, -3.37198, 2.71116, 0.10075],
[4.63394, -3.1044, 2.89785, 0.095],
[4.68944, -2.83268, 2.84466, 0.09749],
[4.73674, -2.55757, 2.57985, 0.1082],
[4.77631, -2.28059, 2.36149, 0.11848],
[4.8075, -2.01313, 2.02551, 0.13294],
[4.84501, -1.75053, 1.75639, 0.15103],
[4.88995, -1.49196, 1.75639, 0.14943],
[4.94334, -1.23803, 1.75639, 0.14773],
[5.00665, -0.98965, 1.75639, 0.14594],
[5.08132, -0.74776, 1.75639, 0.14414],
[5.1708, -0.51451, 1.75639, 0.14224],
[5.27913, -0.29264, 1.96116, 0.1259],
[5.40213, -0.07985, 2.16863, 0.11333],
[5.53703, 0.12554, 2.43487, 0.10092],
[5.68151, 0.32518, 2.68769, 0.09169],
[5.83418, 0.52036, 3.02332, 0.08196],
[5.99377, 0.71235, 2.82652, 0.08833],
[6.15901, 0.9023, 2.28839, 0.11002],
[6.32848, 1.09106, 1.93324, 0.13122],
[6.50098, 1.27919, 1.65342, 0.15437],
[6.68236, 1.47478, 1.43971, 0.18528],
[6.85925, 1.6725, 1.3006, 0.20398],
[7.02716, 1.87438, 1.3, 0.20199],
[7.18173, 2.08222, 1.3, 0.19924],
[7.31814, 2.29769, 1.3, 0.19617],
[7.43038, 2.52235, 1.3, 0.19318],
[7.51119, 2.7569, 1.3, 0.19083],
[7.55339, 2.9997, 1.3, 0.18957],
[7.55655, 3.24599, 1.38069, 0.17839],
[7.52489, 3.49107, 1.49816, 0.16495],
[7.46368, 3.73176, 1.63361, 0.15203],
[7.37783, 3.96628, 1.75342, 0.14243],
[7.27099, 4.19362, 1.75342, 0.14326],
[7.14346, 4.4119, 1.86674, 0.13543],
[6.99819, 4.62056, 1.98218, 0.12827],
[6.83767, 4.81946, 1.83363, 0.1394],
[6.66318, 5.00808, 1.76517, 0.14557],
[6.47594, 5.18601, 1.76517, 0.14633],
[6.27702, 5.35289, 1.76517, 0.1471],
[6.06696, 5.5079, 1.76517, 0.1479],
[5.84611, 5.64984, 1.76517, 0.14873],
[5.61249, 5.77317, 1.76517, 0.14966],
[5.3675, 5.87473, 1.93478, 0.13707],
[5.115, 5.95716, 2.09336, 0.12689],
[4.85736, 6.02278, 2.25731, 0.11778],
[4.59616, 6.07367, 2.35742, 0.11288],
[4.33232, 6.11088, 2.44808, 0.10884],
[4.06654, 6.13528, 2.39377, 0.11149],
[3.79938, 6.14756, 2.15959, 0.12384],
[3.53125, 6.1482, 1.92473, 0.1393],
[3.26255, 6.13764, 1.92473, 0.13971],
[2.9936, 6.11533, 1.92473, 0.14021],
[2.72482, 6.08022, 1.92473, 0.14083],
[2.45677, 6.03098, 1.92473, 0.1416],
[2.19045, 5.9642, 1.92473, 0.14265],
[1.92786, 5.87514, 2.28025, 0.1216],
[1.66894, 5.77009, 2.62876, 0.1063],
[1.41317, 5.65288, 3.03899, 0.09258],
[1.15992, 5.52644, 3.5798, 0.07907],
[0.90856, 5.39326, 4.0, 0.07112],
[0.65844, 5.25567, 4.0, 0.07137],
[0.4089, 5.11597, 4.0, 0.07149],
[0.15316, 4.97507, 4.0, 0.073],
[-0.10383, 4.83629, 3.99395, 0.07313],
[-0.36229, 4.70016, 3.9791, 0.07341],
[-0.62243, 4.56722, 3.9791, 0.07342],
[-0.88449, 4.43815, 3.9791, 0.07341],
[-1.1487, 4.31366, 3.9791, 0.0734],
[-1.41519, 4.19426, 3.9791, 0.07339],
[-1.68397, 4.08021, 3.9791, 0.07338],
[-1.95497, 3.97161, 4.0, 0.07299],
[-2.22805, 3.8683, 4.0, 0.07299],
[-2.503, 3.76993, 4.0, 0.073],
[-2.77955, 3.67599, 4.0, 0.07302],
[-3.05737, 3.58573, 4.0, 0.07303],
[-3.33615, 3.49833, 3.97046, 0.07358],
[-3.61567, 3.41324, 3.37128, 0.08667],
[-3.89575, 3.32994, 2.94071, 0.09936],
[-4.16811, 3.25059, 2.57971, 0.10997],
[-4.4394, 3.16981, 2.28345, 0.12396],
[-4.70842, 3.08598, 2.01337, 0.13996],
[-4.97412, 2.99749, 1.76473, 0.15869],
[-5.23543, 2.90261, 1.53606, 0.18099],
[-5.49124, 2.79947, 1.53606, 0.17956],
[-5.74005, 2.68588, 1.53606, 0.17806],
[-5.97996, 2.55935, 1.53606, 0.17657],
[-6.20828, 2.41682, 1.53606, 0.17522],
[-6.42097, 2.25442, 1.53606, 0.17421],
[-6.61138, 2.06763, 1.67145, 0.15958],
[-6.78128, 1.8621, 1.83673, 0.14518],
[-6.93306, 1.64234, 1.95789, 0.13641],
[-7.06802, 1.41108, 2.07053, 0.12932],
[-7.18723, 1.17049, 2.17024, 0.12372],
[-7.2915, 0.92227, 2.22494, 0.12101],
[-7.38107, 0.66766, 2.32061, 0.11631],
[-7.45672, 0.40801, 2.38208, 0.11353],
[-7.51896, 0.14436, 2.37497, 0.11406],
[-7.56764, -0.1225, 2.37497, 0.11422],
[-7.60263, -0.39175, 2.37497, 0.11432],
[-7.62377, -0.66256, 2.37497, 0.11437],
[-7.63088, -0.93408, 2.37497, 0.11437],
[-7.62381, -1.20545, 2.37497, 0.1143],
[-7.60253, -1.47581, 2.38011, 0.11394],
[-7.56721, -1.74435, 2.29091, 0.11823],
[-7.51843, -2.01045, 2.08174, 0.12995],
[-7.45739, -2.27372, 2.08174, 0.12982],
[-7.38521, -2.53394, 2.01134, 0.13426],
[-7.30144, -2.79049, 1.8059, 0.14944],
[-7.20549, -3.04261, 1.73936, 0.15509],
[-7.09559, -3.28887, 1.73936, 0.15504],
[-6.96929, -3.527, 1.73936, 0.15498],
[-6.8287, -3.75695, 1.73936, 0.15496],
[-6.67165, -3.97622, 1.73936, 0.15506],
[-6.4951, -4.18085, 1.73936, 0.15538],
[-6.2987, -4.36795, 1.82741, 0.14844],
[-6.08539, -4.53775, 2.00865, 0.13573],
[-5.85868, -4.69228, 2.19221, 0.12515],
[-5.62103, -4.83337, 2.38563, 0.11585],
[-5.37428, -4.96263, 2.58923, 0.10758],
[-5.11989, -5.08146, 2.77757, 0.10109],
[-4.85891, -5.19092, 3.00755, 0.0941],
[-4.59233, -5.29214, 3.19509, 0.08925],
[-4.3208, -5.38583, 3.39518, 0.0846],
[-4.04491, -5.47268, 3.60813, 0.08016],
[-3.76518, -5.55332, 3.83247, 0.07596],
[-3.48209, -5.62831, 4.0, 0.07321],
[-3.1961, -5.69816, 4.0, 0.0736],
[-2.90764, -5.76329, 4.0, 0.07393],
[-2.61717, -5.82405, 4.0, 0.07419],
[-2.32511, -5.88065, 4.0, 0.07437]]

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
