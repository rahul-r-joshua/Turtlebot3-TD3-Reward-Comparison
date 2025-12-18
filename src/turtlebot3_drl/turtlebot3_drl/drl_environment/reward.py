#!/usr/bin/env python3
#
# Reward functions for TurtleBot3 DRL
# 1. Hybrid Reward (Combines Goal Progress and Movement)
# 2. Goal-Oriented Reward (Focuses on Reaching the Goal)
# 3. Movement-Based Shaped Reward (Focuses on Smooth Movement)


from ..common.settings import (
    REWARD_FUNCTION,
    COLLISION_OBSTACLE,
    COLLISION_WALL,
    TUMBLE,
    SUCCESS,
    TIMEOUT,
    RESULTS_NUM,
)

goal_dist_initial = 0
reward_function_internal = None

def get_reward(
    succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance
):
    return reward_function_internal(
        succeed, action_linear, action_angular, distance_to_goal, goal_angle, min_obstacle_distance
    )


# ---------------------------- 1. Hybrid Reward ----------------------------
def get_reward_A(
    succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist
):
    

    # --- Yaw alignment penalty [-3.14, 0]
    r_yaw = -1.0 * abs(goal_angle)

    # --- Angular velocity penalty [-4, 0]
    r_vangular = -1.0 * (action_angular**2)

    # --- Goal-oriented distance reward, normalized [-1, 1]
    if goal_dist_initial > 0:
        r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1
    else:
        r_distance = 0.0

    # --- Obstacle penalty (stronger)
    if min_obstacle_dist < 0.22:
        r_obstacle = -30.0
    else:
        r_obstacle = 0.0

    # --- Linear velocity shaping (encourage ~0.22 m/s)
    r_vlinear = -75.0 * ((0.22 - action_linear) ** 2)

    # --- Combine all components
    reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular - 1.0

    # --- Terminal conditions
    if succeed == SUCCESS:
        reward += 2500.0
    elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL):
        reward -= 2000.0

    return float(reward)


# ---------------------------- 2. Goal-Oriented Reward ----------------------------
# def get_reward_A(
#     succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist
# ):

#     # --- Yaw alignment penalty [-3.14, 0]
#     r_yaw = -1.0 * abs(goal_angle)

#     # --- Angular velocity penalty [-4, 0]
#     r_vangular = -1.0 * (action_angular**2)

#     # --- Goal-oriented distance reward, normalized [-1, 1]
#     if goal_dist_initial > 0:
#         r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1
#     else:
#         r_distance = 0.0

#     # --- Obstacle penalty (moderate)
#     if min_obstacle_dist < 0.22:
#         r_obstacle = -20.0
#     else:
#         r_obstacle = 0.0

#     # --- Linear velocity shaping (encourage ~0.22 m/s)
#     r_vlinear = -10.0 * ((0.22 - action_linear) ** 2)

#     # --- Combine all components
#     reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular - 1.0

#     # --- Terminal conditions
#     if succeed == SUCCESS:
#         reward += 2500.0
#     elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL):
#         reward -= 2000.0

#     return float(reward)


# ---------------------------- 3. Movement-Based Shaped Reward ----------------------------

# def get_reward_A(
#     succeed, action_linear, action_angular, goal_dist, goal_angle, min_obstacle_dist
# ):

#     # --- Yaw alignment penalty [-3.14, 0]
#     r_yaw = -1.0 * abs(goal_angle)

#     # --- Angular velocity penalty [-4, 0]
#     r_vangular = -1.0 * (action_angular**2)

#     # --- Goal-oriented distance reward, normalized [-1, 1]
#     if goal_dist_initial > 0:
#         r_distance = (2 * goal_dist_initial) / (goal_dist_initial + goal_dist) - 1
#     else:
#         r_distance = 0.0

#     # --- Obstacle penalty (stronger)
#     if min_obstacle_dist < 0.22:
#         r_obstacle = -20.0
#     else:
#         r_obstacle = 0.0

#     # --- Linear velocity shaping (encourage ~0.22 m/s)
#     r_vlinear = -1.0 * (((0.22 - action_linear) * 10) ** 2)

#     # --- Combine all components
#     reward = r_yaw + r_distance + r_obstacle + r_vlinear + r_vangular - 1.0

#     # --- Terminal conditions
#     if succeed == SUCCESS:
#         reward += 2500.0
#     elif succeed in (COLLISION_OBSTACLE, COLLISION_WALL):
#         reward -= 2000.0

#     return float(reward)

def reward_initalize(init_distance_to_goal):
    """Store initial goal distance for normalization."""
    global goal_dist_initial
    goal_dist_initial = init_distance_to_goal


function_name = "get_reward_" + REWARD_FUNCTION
reward_function_internal = globals().get(function_name)
if reward_function_internal is None:
    quit(f"Error: reward function {function_name} does not exist")

