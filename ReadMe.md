# Deep Reinforcement Learning Navigation on TurtleBot3 Using TD3 with Comparative Reward Design


This repository implements **TD3-based deep reinforcement learning** for **TurtleBot3 navigation**, including a **systematic comparison of reward structures** to evaluate their impact on:
- **Convergence**
- **Stability**
- **Navigation performance**

---

## ✨ Key Features

- 🧠 TD3-based deep reinforcement learning
- 🎯 Comparative reward design study
- 🤖 TurtleBot3 Waffle Pi support
- 🧪 Training & testing in Gazebo
- 🔁 Load and evaluate pre-trained models

---

## 🏆 Reward Structures

The project includes three main reward types used for training the TurtleBot3 agent:

| Reward Type        | Focus                        | Obstacle Penalty | Velocity Shaping | Behavior                   |
| ------------------ | ---------------------------- | ---------------- | ---------------- | -------------------------- |
| **Hybrid**         | Balance goal & smooth motion | -30              | Strong (-75)     | Careful, smooth navigation |
| **Goal-Oriented**  | Reaching the goal            | -20              | Weak (-10)       | Fast but less smooth       |
| **Movement-Based** | Smooth, natural motion       | -20              | Moderate shaping | Safe and steady driving    |


## 🛠️ Prerequisites

Before setting up the environment, ensure the following are installed:

- ✅ Ubuntu 22.04
- ✅ ROS 2 **Humble**  
  [Official ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- ✅ Gazebo (installed with ROS 2)  
- ✅ TurtleBot3 packages  
  Install TurtleBot3 and its dependencies: https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/


### ⚠️ Remove Existing TurtleBot3 Packages

To avoid conflicts, remove any existing TurtleBot3 installations:

```bash
sudo apt remove ros-humble-turtlebot3-msgs
sudo apt remove ros-humble-turtlebot3
```

## 🛠️ Setup Instructions

Follow these steps to set up the TurtleBot3 TD3 environment:

---

### 1️⃣ Clone the Repository

Navigate to your home directory and clone the repository:

```bash
cd ~
git clone https://github.com/rahul-r-joshua/Turtlebot3-TD3-Reward-Comparison.git
```

### 2️⃣ Build the Workspace

Navigate to your ROS 2 workspace and build the packages using `colcon`:

```bash
cd ~/Turtlebot3-TD3-Reward-Comparison
colcon build --symlink-install
```

> ⚠️ **Important:** Make sure you are in the root of your workspace (`~/tb3_ws`) before running `colcon build`. 

> 💡 **Tip:** Use `--symlink-install` during development to apply changes without rebuilding the entire workspace.


## 3️⃣ Source the Workspace

After building the workspace, you need to **source it** to make the packages available in your terminal.

### Temporary Sourcing (Per Terminal)

Run this command in every new terminal session:

```bash
source install/setup.bash
```
### Permanent Sourcing (Automatic in Every Terminal)

To automatically source the workspace whenever you open a terminal, add the following to your `.bashrc`:

```bash
echo "source ~/Turtlebot3-TD3-Reward-Comparison/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
> ⚠️ **Important:** Sourcing is required in every terminal before running ROS 2 commands.

> 💡 **Tip:** Permanent sourcing ensures you don’t have to manually source every time.


## 🚦 Running the Simulation

> ⚠️ Each command should be executed in a **new terminal**.

---

## 💡 Note on Reward Structures

The reward definitions are located in:
```bash 
cd ~/Turtlebot3-TD3-Reward-Comparison/src/turtlebot3_drl/turtlebot3_drl/drl_environment/
```
```bash
ls
```
``` bash
reward.py 
```

- All reward structures are included in this file.  
- **Important:** Two reward functions are commented out by default.  
- If you want to train using a specific reward:
  1. **Uncomment** the desired reward function.
  2. **Comment out** the other reward functions to avoid conflicts.


### 1️⃣ Launch Gazebo Simulation

Start the Gazebo simulation:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_drl_stage9.launch.py
```

> 💡 **Tip:** If Gazebo gets stuck, run:

```bash
source /usr/share/gazebo/setup.bash
```


### 2️⃣ Run the DRL Environment

In a new terminal, start the DRL environment:

```bash
ros2 run turtlebot3_drl environment
```

### 3️⃣ Run the Goal Publisher

In a new terminal, start the goal publisher:

```bash
ros2 run turtlebot3_drl gazebo_goals
```

### 4️⃣ Train an Agent

Train the agent using **TD3**:

```bash
ros2 run turtlebot3_drl train_agent td3
```


### 5️⃣ Test Trained Weights

To test a trained agent, run:

```bash
ros2 run turtlebot3_drl test_agent td3 <saved weights folder name> <episode>
```

### 🏁 Pre-Trained Weights


To use the pre-trained weights included in this repository, follow these steps:

**Launch the Gazebo simulation** in a new terminal:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_drl_stage9.launch.py
```

**Run the DRL environment** in a new terminal:

   ```bash
   ros2 run turtlebot3_drl environment
   ```

**Start the goal publisher** in a new terminal:

   ```bash
   ros2 run turtlebot3_drl gazebo_goals
   ```

### 🚀 Load and Test the Rewards


### 🟡 Hybrid Reward

```bash
ros2 run turtlebot3_drl test_agent td3 /home/{User_Name}/Turtlebot3-TD3-Reward-Comparison/src/turtlebot3_drl/trained_reward/td3_2_stage_9 2300
```


### 🟢 Goal-Oriented Reward
```bash
ros2 run turtlebot3_drl test_agent td3 /home/{User_Name}/Turtlebot3-TD3-Reward-Comparison/src/turtlebot3_drl/trained_reward/td3_6_stage_9 3000
```


### 🔵 Movement-Based Shaped Reward
```bash
ros2 run turtlebot3_drl test_agent td3 /home/{User_Name}/Turtlebot3-TD3-Reward-Comparison/src/turtlebot3_drl/trained_reward/td3_8_stage_9 2800
```

> ⚠️ **Note:**  There are multiple trained weights available for each reward type in this repository.  
> The examples above use suggested episodes for quick testing, but you can experiment with other weights by modifying the **episode number** or selecting different weight folders.

## 📦 Pre-Trained Weights

The pre-trained TD3 weights for **Hybrid**, **Goal-Oriented**, and **Movement-Based** rewards are available in the [Releases](https://github.com/rahul-r-joshua/Turtlebot3-TD3-Reward-Comparison/releases) section.  

You can directly download the `.zip` file and use it for testing your agent in Gazebo.  

> 💡 **Note:** After downloading, **extract the `.zip` file** and move its contents to your TurtleBot3 DRL directory:  


```bash
unzip ~/Downloads/trained_reward_check.zip -d ~/Turtlebot3-TD3-Reward-Comparison/src/turtlebot3_drl/
```



## 📊 Training Insights & Reward Comparison

### Overfitting Consideration
During training, it is important to **stop once the agent achieves good performance**. Continuing training for too long may lead to **overfitting**, where the agent performs well in the training environment but fails to generalize to new scenarios.

### Reward-Based Performance
Based on experiments with different reward structures:

| Reward Type                  | Success Rate (%) |
|-------------------------------|----------------|
| 🟡 Hybrid Reward              | 82 - 87       |
| 🟢 Goal-Oriented Reward       | 70 - 75       |
| 🔵 Movement-Based Shaped Reward | 55 - 60       |

> 💡 **Observation:** Hybrid reward performed best, likely due to combining both goal-directed and movement-based incentives, which helps the agent learn a balanced navigation strategy.



## 👨‍💻 Author

**Rahul R**  
Deep Reinforcement Learning | Robotics | ROS 2
