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
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo apt update
sudo apt install portaudio19-dev python3-pyaudio -y
sudo apt install ffmpeg -y
echo 'export OPENAI_API_KEY=your_openai_api_key_here' >> ~/.bashrc
echo 'export ELEVENLABS_API_KEY=your_api_key_not_required_but_fun' >> ~/.bashrc
source ~/.bashrc
cd ~/turtlebot3_ws/src/
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/brumePyvault/llm_mobile_robot.git
cd ~/turtlebot3_ws
colcon build
```

### Step Two

#### Open terminal 1

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### Open terminal 2

##### Initial pose of the robot

For the sake of this project, I have already done the mapping and put the virtual map in the turtle_world folder.

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/turtlebot3_ws/src/llm_mobile_robot/turtle_world/turtle_world.yaml
```

![Initial pose of the robot](https://res.cloudinary.com/deuhrgf1w/image/upload/v1779575451/1d3c480b-c455-4037-943e-f0dadbb01657.png)

After the virtual enviroment has launched, we have to [localize](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#estimate-initial-pose) the robot on the virtual map.
only do the estimate initial pose instruction

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
