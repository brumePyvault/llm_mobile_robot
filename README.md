# LLM Mobile Robot

This repository implements an LLM-based command-processing system for controlling a mobile robot using natural language commands in a ROS 2 simulation environment. The system translates typed user commands into executable Python robot policies, validates the generated code for safety, and executes approved actions through a controlled RobotAPI.
The project forms part of the thesis:
Evaluation of LLM-Based Command Processing Strategies for Natural Language Control of Mobile Robots

## Setup

[Install ROS](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

[Read the pc setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

### Step One

Clone necessary git repos

```bash
pip install -U pip
pip install -e .
pip install requests SpeechRecognition elevenlabs pyyaml
python3 -m pip install openai
sudo apt install portaudio19-dev python3-pyaudio -y
sudo apt install ffmpeg -y
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/brumePyvault/llm_mobile_robot.git
cd ~/turtlebot3_ws
colcon build --symlink-install
```

### Step Two

#### Open terminal 1

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### Open terminal 2

##### Initial pose of the robot

![Initial pose of the robot](https://res.cloudinary.com/deuhrgf1w/image/upload/v1779566849/ca67bd04-65e5-4068-8b43-44bd4eaf5b42.png)

After the virtual enviroment has launched, we have to [localize](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#estimate-initial-pose) the robot on the virtual map.
only do the estimate initial pose instruction

##### After localization

For the sake of this project, I have already done the mapping and put the virtual map in the turtle_world folder.

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/turtlebot3_ws/src/llm_mobile_robot/turtle_world/turtle_world.yaml
```

#### Open terminal 3

```bash
ros2 launch llm_mobile_robot start_llm_control.launch.py
```

#### Open terminal 4

**this is a push to talk STT(Speech to Text)**

```bash
ros2 run llm_mobile_robot stt
```

### Step three

Enjoy talking to terminal 4 and watch the movement on gazebo

## Authors

- [@brumePyvault](https://www.github.com/brumepyvault)
