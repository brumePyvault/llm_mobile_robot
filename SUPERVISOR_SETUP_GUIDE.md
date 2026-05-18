# LLM Mobile Robot – Supervisor Quick Start (ROS 2 Cloud / The Construct)

This guide explains how to run this project in an online ROS 2 Linux environment (like **The Construct**).

---

## 1) What your supervisor needs before starting

- A The Construct account (or similar ROS 2 cloud Linux workspace).
- This project folder uploaded or cloned into that workspace.
- An OpenAI API key (for LLM command generation).

---

## 2) Open a ROS 2 workspace in the cloud

1. Start a ROS 2 environment in The Construct.
2. Choose your ROS 2 distro (the same one you used during development, if possible).
3. Open a terminal in that cloud environment.

---

## 3) Put this project in the ROS 2 workspace

If needed, copy/clone this folder into your workspace `src` directory.

Example target layout:

```bash
~/ros2_ws/src/llm_mobile_robot
```

---

## 4) Install dependencies

### 4.1 Python package dependencies

From the package folder (or workspace root), install required Python modules:

```bash
pip3 install -U python-dotenv openai
```

### 4.2 ROS dependencies

From workspace root (`~/ros2_ws`):

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 5) TurtleBot + Gazebo simulation packages (important)

Because this project is intended for **TurtleBot simulation in Gazebo** (not TurtleSim), ensure TurtleBot simulation packages are installed in your cloud ROS image.

Typical packages to check/install (depends on ROS distro):

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-turtlebot3 \
  ros-$ROS_DISTRO-turtlebot3-msgs \
  ros-$ROS_DISTRO-turtlebot3-gazebo
```

If your cloud environment does not allow `sudo`, use the platform package manager UI or prebuilt image with TurtleBot3/Gazebo already included.

Also set model type when needed:

```bash
export TURTLEBOT3_MODEL=burger
```

---

## 6) Build the workspace

From workspace root (`~/ros2_ws`):

```bash
colcon build --symlink-install
```

Then source:

```bash
source install/setup.bash
```

---

## 7) Configure environment file for API key

This project loads environment variables via `.env`.

1. Copy template if needed:

```bash
cp src/llm_mobile_robot/.env.example src/llm_mobile_robot/.env
```

2. Edit `.env` and set your OpenAI API key, for example:

```env
OPENAI_API_KEY=your_key_here
```

---

## 8) Run simulation + robot controller (two terminals)

> You said commands are run manually in separate terminals. Use this exact flow.

### Terminal A: Start TurtleBot3 Gazebo simulation

```bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal B: Start this project node

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch llm_mobile_robot start_llm_control.launch.py
```

---

## 9) Basic verification

In a new terminal (after sourcing workspace):

```bash
ros2 node list
```

You should see the `llm_control` node running.

You can also inspect topics:

```bash
ros2 topic list
```

---

## 10) Troubleshooting

- **`Package 'llm_mobile_robot' not found`**
  - Rebuild and re-source:
    - `colcon build --symlink-install`
    - `source install/setup.bash`
- **OpenAI/authentication errors**
  - Confirm `.env` exists and `OPENAI_API_KEY` is valid.
- **TurtleBot launch package missing**
  - Install TurtleBot3 + Gazebo packages for your ROS distro.
- **Gazebo not opening in cloud**
  - Use the platform GUI/simulator tab (some cloud IDEs do not pop up desktop windows in terminal-only mode).

---

## 11) One-line run order summary (for supervisor)

1. Open ROS 2 cloud workspace.
2. Ensure TurtleBot3/Gazebo packages are installed.
3. Build (`colcon build`) and source (`source install/setup.bash`).
4. Terminal A: launch TurtleBot3 Gazebo world.
5. Terminal B: launch `llm_mobile_robot` node.

