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
        racing_track = [[0.00307, -3.35292, 4.0, 0.07534],
[0.26724, -3.49795, 4.0, 0.07534],
[0.53142, -3.64289, 4.0, 0.07533],
[0.79563, -3.7875, 4.0, 0.0753],
[1.05985, -3.93178, 4.0, 0.07526],
[1.3241, -4.07573, 4.0, 0.07523],
[1.58838, -4.21927, 4.0, 0.07519],
[1.85269, -4.36231, 4.0, 0.07513],
[2.11705, -4.50447, 4.0, 0.07504],
[2.38149, -4.64543, 4.0, 0.07492],
[2.64602, -4.78488, 4.0, 0.07476],
[2.91065, -4.92248, 4.0, 0.07457],
[3.1754, -5.05783, 4.0, 0.07434],
[3.44028, -5.19047, 3.62734, 0.08167],
[3.7053, -5.31988, 3.23146, 0.09127],
[3.97048, -5.44542, 2.86443, 0.10243],
[4.23581, -5.56631, 2.52935, 0.11528],
[4.50129, -5.68151, 2.22704, 0.12995],
[4.76688, -5.79008, 1.95189, 0.147],
[5.03253, -5.89036, 1.7102, 0.16603],
[5.29813, -5.9805, 1.48444, 0.18894],
[5.56345, -6.05817, 1.31544, 0.21016],
[5.82813, -6.12049, 1.31544, 0.20671],
[6.09151, -6.1639, 1.31544, 0.20293],
[6.3525, -6.18394, 1.31544, 0.19899],
[6.6092, -6.17516, 1.31544, 0.19525],
[6.85805, -6.13017, 1.31544, 0.19224],
[7.09267, -6.04162, 1.41788, 0.17687],
[7.31081, -5.91759, 1.58444, 0.15837],
[7.51274, -5.76653, 1.71885, 0.14671],
[7.69851, -5.59372, 1.83678, 0.13813],
[7.86812, -5.40315, 1.95213, 0.13068],
[8.0218, -5.19821, 2.06856, 0.12384],
[8.16008, -4.98176, 2.15658, 0.1191],
[8.28329, -4.75593, 2.24001, 0.11484],
[8.39185, -4.52254, 2.29657, 0.11208],
[8.48597, -4.28298, 2.1895, 0.11755],
[8.56603, -4.03855, 2.00277, 0.12843],
[8.63209, -3.79029, 1.83102, 0.1403],
[8.68423, -3.53922, 1.65186, 0.15524],
[8.7221, -3.28623, 1.46835, 0.17422],
[8.74542, -3.03222, 1.3, 0.19621],
[8.75182, -2.77818, 1.3, 0.19547],
[8.73822, -2.5257, 1.3, 0.1945],
[8.70122, -2.27715, 1.3, 0.1933],
[8.63645, -2.03611, 1.3, 0.19199],
[8.53825, -1.80822, 1.3, 0.19087],
[8.40031, -1.6024, 1.32345, 0.18722],
[8.22701, -1.42296, 1.46601, 0.17016],
[8.02677, -1.26862, 1.63708, 0.15444],
[7.80579, -1.13685, 1.84891, 0.13915],
[7.56891, -1.02455, 2.05819, 0.12737],
[7.31936, -0.92928, 2.3006, 0.11611],
[7.0597, -0.84867, 2.6032, 0.10444],
[6.79217, -0.78022, 3.01648, 0.09155],
[6.51876, -0.72127, 3.67815, 0.07604],
[6.24138, -0.66897, 4.0, 0.07057],
[5.96171, -0.6206, 4.0, 0.07096],
[5.67651, -0.57339, 3.9525, 0.07314],
[5.39094, -0.52336, 3.70893, 0.07817],
[5.10558, -0.46981, 3.50465, 0.08284],
[4.82108, -0.4121, 3.33567, 0.08703],
[4.53794, -0.34965, 3.19821, 0.09066],
[4.25671, -0.28164, 3.08811, 0.0937],
[3.97785, -0.20735, 3.00094, 0.09616],
[3.70185, -0.12603, 2.93278, 0.09811],
[3.42922, -0.03697, 2.88074, 0.09956],
[3.16048, 0.0605, 2.84311, 0.10055],
[2.89617, 0.16691, 2.81926, 0.10106],
[2.63684, 0.28274, 2.80948, 0.10109],
[2.38303, 0.4083, 2.80948, 0.10079],
[2.13526, 0.54381, 2.80948, 0.10052],
[1.89404, 0.68937, 2.80948, 0.10028],
[1.65983, 0.84499, 2.80948, 0.10009],
[1.43303, 1.01057, 2.80948, 0.09995],
[1.214, 1.18592, 2.81479, 0.09968],
[1.00297, 1.37075, 2.83691, 0.09888],
[0.80009, 1.56469, 2.87831, 0.09751],
[0.60537, 1.76728, 2.94252, 0.0955],
[0.4187, 1.978, 3.03502, 0.09275],
[0.2398, 2.1962, 3.16514, 0.08915],
[0.06822, 2.42121, 3.34828, 0.08451],
[-0.09667, 2.6522, 3.60581, 0.07871],
[-0.25563, 2.88828, 3.95689, 0.07193],
[-0.40946, 3.12834, 4.0, 0.07128],
[-0.55864, 3.37081, 4.0, 0.07117],
[-0.70292, 3.61296, 3.40524, 0.08278],
[-0.84137, 3.85085, 2.89205, 0.09517],
[-0.97433, 4.08255, 2.54253, 0.10507],
[-1.09834, 4.29751, 2.1848, 0.11359],
[-1.22451, 4.5106, 1.90949, 0.12969],
[-1.35426, 4.72057, 1.88117, 0.13121],
[-1.489, 4.9262, 1.75618, 0.13999],
[-1.63057, 5.12588, 1.75618, 0.13938],
[-1.78084, 5.31793, 1.75618, 0.13886],
[-1.94275, 5.49969, 1.75618, 0.1386],
[-2.11975, 5.66777, 1.75618, 0.13899],
[-2.31223, 5.82119, 1.75618, 0.14016],
[-2.52232, 5.95657, 1.87502, 0.13329],
[-2.74753, 6.07463, 2.0342, 0.125],
[-2.98521, 6.17662, 2.17219, 0.11907],
[-3.23337, 6.26337, 2.33133, 0.11276],
[-3.49006, 6.33591, 2.42774, 0.10987],
[-3.75395, 6.3945, 2.52049, 0.10725],
[-4.02367, 6.43941, 2.60597, 0.10493],
[-4.29784, 6.47093, 2.68389, 0.10283],
[-4.57513, 6.48938, 2.75599, 0.10083],
[-4.85423, 6.4952, 2.62516, 0.10634],
[-5.134, 6.48902, 2.44097, 0.11464],
[-5.41342, 6.47114, 2.28846, 0.12235],
[-5.69154, 6.44185, 2.01171, 0.13901],
[-5.96733, 6.40066, 1.7884, 0.15592],
[-6.23972, 6.34714, 1.57373, 0.17639],
[-6.50744, 6.28043, 1.57373, 0.17532],
[-6.76886, 6.19904, 1.57373, 0.17398],
[-7.02229, 6.10164, 1.57373, 0.17252],
[-7.26448, 5.98454, 1.57373, 0.17094],
[-7.49128, 5.8439, 1.57373, 0.16958],
[-7.69612, 5.67502, 1.76683, 0.15025],
[-7.88158, 5.48571, 1.89301, 0.14],
[-8.04868, 5.28022, 1.99847, 0.13253],
[-8.19809, 5.06176, 2.08949, 0.12667],
[-8.33039, 4.8329, 2.17877, 0.12133],
[-8.44628, 4.59579, 2.16385, 0.12196],
[-8.54635, 4.35212, 2.04806, 0.12862],
[-8.63072, 4.10315, 1.91138, 0.13753],
[-8.69942, 3.84999, 1.7207, 0.15245],
[-8.75235, 3.5937, 1.54364, 0.16953],
[-8.78901, 3.33527, 1.36872, 0.19071],
[-8.80757, 3.07572, 1.36872, 0.19011],
[-8.80601, 2.81646, 1.36872, 0.18942],
[-8.78163, 2.55934, 1.36872, 0.18869],
[-8.72963, 2.30723, 1.36872, 0.18807],
[-8.64417, 2.06446, 1.36872, 0.18803],
[-8.5173, 1.83819, 1.71083, 0.15163],
[-8.36425, 1.62491, 1.92855, 0.13612],
[-8.19032, 1.4231, 2.12433, 0.12541],
[-7.99877, 1.23168, 2.32808, 0.11632],
[-7.79214, 1.04967, 2.55343, 0.10784],
[-7.5726, 0.87613, 2.80659, 0.09971],
[-7.34208, 0.7101, 3.10927, 0.09137],
[-7.10237, 0.55056, 3.47707, 0.08281],
[-6.85513, 0.39649, 3.98401, 0.07312],
[-6.602, 0.24674, 4.0, 0.07353],
[-6.3444, 0.10023, 4.0, 0.07409],
[-6.08347, -0.04397, 4.0, 0.07453],
[-5.82024, -0.18676, 4.0, 0.07487],
[-5.55542, -0.32881, 4.0, 0.07513],
[-5.28974, -0.47084, 4.0, 0.07532],
[-5.02413, -0.61304, 4.0, 0.07532],
[-4.75866, -0.75549, 4.0, 0.07532],
[-4.4933, -0.89817, 4.0, 0.07532],
[-4.22805, -1.04108, 4.0, 0.07532],
[-3.96293, -1.18422, 4.0, 0.07532],
[-3.69792, -1.32759, 4.0, 0.07533],
[-3.43303, -1.4712, 4.0, 0.07533],
[-3.16825, -1.61503, 4.0, 0.07533],
[-2.90359, -1.7591, 4.0, 0.07533],
[-2.63905, -1.9034, 4.0, 0.07533],
[-2.37463, -2.04794, 4.0, 0.07534],
[-2.11033, -2.19271, 4.0, 0.07534],
[-1.84615, -2.33773, 4.0, 0.07534],
[-1.58198, -2.48276, 4.0, 0.07534],
[-1.3178, -2.62779, 4.0, 0.07534],
[-1.05363, -2.77282, 4.0, 0.07534],
[-0.78946, -2.91785, 4.0, 0.07534],
[-0.52528, -3.06287, 4.0, 0.07534],
[-0.26111, -3.2079, 4.0, 0.07534]]

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
