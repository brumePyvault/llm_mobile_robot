# LLM Mobile Robot Agent Harness

This project implements an **LLM-driven mobile robot agent loop** in ROS 2. It is designed as a practical agent harness: the robot observes world state, receives a goal (voice/text command), asks an LLM to generate an action policy, and executes that policy through a constrained robot API.

It is a strong fit for the Humanoid internship challenge because it demonstrates:
- an environment the agent acts in (mapped robot world with waypoints),
- structured observations (robot pose + known waypoint context),
- a defined action space (navigation, speech, save waypoint, drive, stop),
- an LLM reasoning/action loop (command → policy code → execution), and
- goal completion (multi-step navigation tasks).

---

## Repository Structure

- `llm_mobile_robot/llm_control.py` — main LLM control node (policy generation + execution loop).
- `llm_mobile_robot/robot.py` — robot action API used by generated policies.
- `llm_mobile_robot/stt.py` — speech-to-text input node (push-to-talk style: key to start/stop recording).
- `llm_mobile_robot/waypoint_store.py` — JSON waypoint loading/saving.
- `launch/start_llm_control.launch.py` — launch file for LLM control node.
- `.env.example` — environment variable template.

---

## How the Agent Harness Works

1. **Input goal** arrives on `/voice/text` (from STT) or can be published manually.
2. `llm_control` gathers **observation context**, including robot pose and known waypoints/distances.
3. The LLM returns Python policy code (`run(robot)`) using only approved robot actions.
4. The code is validated and executed.
5. The robot performs actions (navigate, speak, etc.) to complete the goal.

This loop directly matches the “observe → reason → act” pattern requested in the challenge.

---

## Prerequisites

- Ubuntu with ROS 2 installed (tested conceptually with a ROS 2 Python package layout).
- Python 3.10+
- A running navigation stack providing:
  - `/amcl_pose`
  - `navigate_to_pose` action server
  - `/cmd_vel`
- API keys:
  - OpenAI (required)
  - ElevenLabs (required if using speech input/output)

---

## Setup

### 1) Clone and enter workspace

```bash
git clone <your-public-repo-url>
cd llm_mobile_robot
```

### 2) Install Python dependencies

```bash
pip install -U pip
pip install -e .
pip install requests SpeechRecognition elevenlabs pyyaml
```

### 3) Configure environment variables

```bash
cp .env.example .env
```

Edit `.env` and set at least:

```env
OPENAI_API_KEY=...
ELEVENLABS_API_KEY=...
OPENAI_MODEL=gpt-4.1-mini
VOICE_TOPIC=/voice/text
WAYPOINTS_FILE=/absolute/path/to/waypoints.json
```

> `WAYPOINTS_FILE` should point to your waypoint JSON file used by navigation.

### 4) Build/source as ROS 2 package (if used inside a ROS 2 workspace)

From workspace root:

```bash
colcon build --packages-select llm_mobile_robot
source install/setup.bash
```

---

## Running the System

Open separate terminals (all with ROS environment sourced and `.env` loaded).

### Terminal A — Bring up robot/nav stack

Run your simulator/robot and localization/nav2 stack so pose and navigation topics are available.

### Terminal B — Start LLM control node

Option 1 (launch file):

```bash
ros2 launch llm_mobile_robot start_llm_control.launch.py
```

Option 2 (direct executable):

```bash
ros2 run llm_mobile_robot llm_control
```

### Terminal C — Start STT voice command input (optional)

```bash
python3 -m llm_mobile_robot.stt
```

**Push-to-talk behavior (your STT flow):**
- Press any key to **start** recording.
- Speak command (e.g., “go to the office, then kitchen”).
- Press any key again to **stop** recording and transcribe.
- Transcript is published to `/voice/text`.

### Terminal C alternative — send typed command without STT

```bash
ros2 topic pub --once /voice/text std_msgs/msg/String "{data: 'Go to the office then the kitchen'}"
```

---

## Example Goal-Directed Tasks

Try commands such as:
- “Go to the office.”
- “Visit the printer, then the meeting room, then stop.”
- “Go to the farthest location first, then the closest remaining location.”

These demonstrate planning over waypoints and multi-step action execution.

---

## Notes on Design Choices

- **Observation representation:** Current pose + known waypoint coordinates/distances gives the LLM enough structured situational awareness for navigation decisions.
- **Action space design:** The generated policy is constrained to a safe/high-level API (`navigate_to`, `wait_until_navigate_done`, `stop`, `say`, `save_waypoint`, `drive`, `come_back`) instead of arbitrary code.
- **Execution safety:** Policy format is restricted to a `run(robot)` function and validated before execution.
- **Real-world practicality:** Works with ROS topics/actions and supports both voice and text command modes.

---

## What to Include in Submission

For your application repository submission, include:
- This codebase.
- This README with setup/run steps.
- A short run log or screen recording showing command → action → task completion.
- (Optional) a short reflection on what worked and what you’d improve.

