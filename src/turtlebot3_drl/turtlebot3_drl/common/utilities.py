#!/usr/bin/env python3
"""
ROS 2 Humble Environment Interface for TurtleBot3 DRL Navigation
Compatible with: Burger, Waffle, Waffle Pi

This module provides Gazebo and DRL environment utility functions
for Reinforcement Learning agents like TD3, DDPG, etc.
"""

from queue import Empty
from turtlebot3_msgs.srv import DrlStep, Goal
from std_srvs.srv import Empty as EmptySrv
import os
import time
import rclpy
import torch
import numpy as np
import glob
import xml.etree.ElementTree as ET

from ..common.settings import (
    REWARD_FUNCTION,
    COLLISION_OBSTACLE,
    COLLISION_WALL,
    TUMBLE,
    SUCCESS,
    TIMEOUT,
    RESULTS_NUM,
)

# --- Stage handling (optional fallback) ---
try:
    with open('/tmp/drlnav_current_stage.txt', 'r') as f:
        stage = int(f.read())
except FileNotFoundError:
    stage = 1
    print("\033[1m\033[93mMake sure to launch the Gazebo simulation node first!\033[0m")

# =========================================================
#                 DEVICE MANAGEMENT
# =========================================================

def check_gpu():
    """Check if CUDA GPU is available."""
    print("Torch CUDA available:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("Device name:", torch.cuda.get_device_name(0))
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")

# =========================================================
#                 ENVIRONMENT CONTROL
# =========================================================

def step(agent_self, action, previous_action):
    """Perform one environment step via DrlStep ROS2 service."""
    req = DrlStep.Request()
    req.action = action
    req.previous_action = previous_action

    while not agent_self.step_comm_client.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('Env step service not available, waiting...')
    future = agent_self.step_comm_client.call_async(req)

    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            if future.result() is not None:
                res = future.result()
                return res.state, res.reward, res.done, res.success, res.distance_traveled
            else:
                agent_self.get_logger().error(
                    f'Exception while calling step service: {future.exception()}'
                )
                print("ERROR getting step service response!")
                return None, 0.0, True, False, 0.0


def init_episode(agent_self):
    """Reset the environment and get initial state."""
    state, _, _, _, _ = step(agent_self, [], [0.0, 0.0])
    return state


def get_goal_status(agent_self):
    """Query if a new goal is available."""
    req = Goal.Request()
    while not agent_self.goal_comm_client.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('Goal service not available, waiting...')
    future = agent_self.goal_comm_client.call_async(req)

    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            if future.result() is not None:
                return future.result().new_goal
            else:
                agent_self.get_logger().error(
                    f'Exception while calling goal service: {future.exception()}'
                )
                print("ERROR getting goal service response!")
                return False


def wait_new_goal(agent_self):
    """Wait until a new goal is generated."""
    while not get_goal_status(agent_self):
        print("Waiting for new goal... (if persists: reset gazebo_goals node)")
        time.sleep(1.0)


def pause_simulation(agent_self, real_robot=False):
    """Pause the Gazebo simulation."""
    if real_robot:
        return
    while not agent_self.gazebo_pause.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('Pause Gazebo service not available, waiting...')
    future = agent_self.gazebo_pause.call_async(EmptySrv.Request())
    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            return


def unpause_simulation(agent_self, real_robot=False):
    """Unpause the Gazebo simulation."""
    if real_robot:
        return
    while not agent_self.gazebo_unpause.wait_for_service(timeout_sec=1.0):
        agent_self.get_logger().info('Unpause Gazebo service not available, waiting...')
    future = agent_self.gazebo_unpause.call_async(EmptySrv.Request())
    while rclpy.ok():
        rclpy.spin_once(agent_self)
        if future.done():
            return


def translate_outcome(outcome):
    """Translate outcome integer codes into readable strings."""
    if outcome == SUCCESS:
        return "SUCCESS"
    elif outcome == COLLISION_WALL:
        return "COLL_WALL"
    elif outcome == COLLISION_OBSTACLE:
        return "COLL_OBST"
    elif outcome == TIMEOUT:
        return "TIMEOUT"
    elif outcome == TUMBLE:
        return "TUMBLE"
    else:
        return f"UNKNOWN({outcome})"

# =========================================================
#                 ORIENTATION HELPERS
# =========================================================

def euler_from_quaternion(quat):
    """Convert quaternion (x, y, z, w) to Euler roll, pitch, yaw."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion (x, y, z, w)."""
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quaternion_from_euler(roll, pitch, yaw):
    """Alias for euler_to_quaternion."""
    return euler_to_quaternion(roll, pitch, yaw)

# =========================================================
#                 MODEL & WORLD UTILITIES
# =========================================================

def detect_robot_model():
    """
    Detect whether 'burger', 'waffle', or 'waffle_pi' model exists.
    """
    current_dir = os.path.dirname(os.path.abspath(__file__))
    while current_dir and not os.path.exists(os.path.join(current_dir, 'src')):
        current_dir = os.path.dirname(current_dir)
    gazebo_paths = glob.glob(
        os.path.join(current_dir, 'src', '**', 'turtlebot3_simulations', 'turtlebot3_gazebo'),
        recursive=True,
    )
    if not gazebo_paths:
        raise FileNotFoundError("Could not find turtlebot3_gazebo in workspace.")
    model_dir = os.path.join(gazebo_paths[0], 'models')
    for model in ['turtlebot3_waffle_pi', 'turtlebot3_waffle', 'turtlebot3_burger']:
        if os.path.exists(os.path.join(model_dir, model)):
            print(f"[INFO] Detected TurtleBot3 model: {model}")
            return model
    raise FileNotFoundError("No TurtleBot3 model found (burger/waffle/waffle_pi).")


def get_scan_count():
    """Extract the LIDAR scan sample count from model.sdf."""
    model_name = detect_robot_model()
    current_dir = os.path.dirname(os.path.abspath(__file__))
    while current_dir and not os.path.exists(os.path.join(current_dir, 'src')):
        current_dir = os.path.dirname(current_dir)
    gazebo_path = glob.glob(
        os.path.join(current_dir, 'src', '**', 'turtlebot3_simulations', 'turtlebot3_gazebo'),
        recursive=True,
    )[0]
    model_path = os.path.join(gazebo_path, 'models', model_name, 'model.sdf')
    tree = ET.parse(model_path)
    root = tree.getroot()
    for link in root.find('model').findall('link'):
        if link.get('name') == 'base_scan':
            samples = int(link.find('sensor').find('ray').find('scan').find('horizontal').find('samples').text)
            print(f"[INFO] {model_name} LIDAR scan samples: {samples}")
            return samples
    raise ValueError("Could not find base_scan in model.sdf")


def get_simulation_speed(stage):
    """(Optional) Get Gazebo world simulation speed factor."""
    try:
        base_path = os.getenv('DRLNAV_BASE_PATH', os.getcwd())
        world_path = os.path.join(
            base_path,
            f'src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_drl_stage{stage}/burger.model',
        )
        tree = ET.parse(world_path)
        root = tree.getroot()
        return float(root.find('world').find('physics').find('real_time_factor').text)
    except Exception:
        return 1.0  # Default simulation speed

# =========================================================
#                     END OF FILE
# =========================================================
