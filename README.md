# ROSA AI Agents with URDF Robot Files

Natural language control for ROS2 robots using ROSA (Robot Operating System Agent).

This repository contains:

- **rosa_agents/** - AI agent implementations for natural language robot control
  - **turtlesim_agent/** - Control ROS2 turtlesim with natural language
  - **robot_URDF_agent/** - Control any URDF robot (Gazebo, RViz, or real hardware)
  - Shared configuration (azure_setup.py, .env.example, requirements.txt)
  
- **robot_files/** - URDF robot packages from ros2_ws
  - **my_robot_description/** - Robot URDF, meshes, and configuration
  - **my_robot_bringup/** - Launch files for robot simulation/hardware

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 or later
- ROS2 Jazzy (or Humble/Iron)
- Python 3.10+
- Azure OpenAI API key (or OpenAI API key)

### Installation

```bash
# 1. Clone the repository
git clone https://github.com/Hypeviper1/ROS2_Ai.git
cd rosa_ws

# 2. Install ROS2 dependencies
sudo apt update
sudo apt install ros-jazzy-turtlesim  # For turtlesim agent

# 3. Create Python virtual environment
python3 -m venv rosa_venv
source rosa_venv/bin/activate

# 4. Install Python dependencies
pip install --upgrade pip
cd rosa_agents
pip install -r requirements.txt

# 5. Configure API keys
cp .env.example .env
nano .env  # Add your Azure OpenAI API keys
```

### Configure Environment Variables

Create a `.env` file in `rosa_agents/`:

```bash
# Azure OpenAI (recommended)
AZURE_OPENAI_API_KEY=your_azure_api_key_here
AZURE_OPENAI_ENDPOINT=https://your-resource.openai.azure.com/
AZURE_DEPLOYMENT_NAME=your_deployment_name
OPENAI_API_VERSION=2024-02-15-preview

# OR OpenAI (alternative)
# OPENAI_API_KEY=your_openai_api_key_here
```



## 🐢 Running TurtleSim Agent

```bash
# Terminal 1: Start turtlesim
source /opt/ros/jazzy/setup.bash
ros2 run turtlesim turtlesim_node

# Terminal 2: Run agent
cd rosa_ws/rosa_agents
source /opt/ros/jazzy/setup.bash
source ../rosa_venv/bin/activate
cd turtlesim_agent
python3 run_rosa.py
```

**Example commands:**
- "Move forward 2 meters"
- "Draw a square with 2 unit sides"
- "Rotate 90 degrees left"
- "Spawn a new turtle at (5, 5)"

## 🤖 Running Robot URDF Agent

### Option 1: Use Provided Robot Files

```bash
# Terminal 1: Build and launch the robot from robot_files/
cd rosa_ws/robot_files
# (Set up a ROS2 workspace if not already done)
colcon build
source install/setup.bash
ros2 launch my_robot_bringup my_robot_launch.py  # Adjust launch file name

# Terminal 2: Run agent
cd rosa_ws/rosa_agents
source /opt/ros/jazzy/setup.bash
source ../rosa_venv/bin/activate
cd robot_URDF_agent
python3 run_robot.py
```

### Option 2: Use Your Own Robot

```bash
# Terminal 1: Launch your robot (Gazebo/RViz/Hardware)
source /opt/ros/jazzy/setup.bash
source ~/your_robot_ws/install/setup.bash
ros2 launch <your_robot_pkg> <launch_file>.py

# Terminal 2: Run agent
cd rosa_ws/rosa_agents
source /opt/ros/jazzy/setup.bash
source ../rosa_venv/bin/activate
cd robot_URDF_agent
python3 run_robot.py
```

**Example commands:**
- "Move forward 1 meter"
- "Rotate 45 degrees right"
- "What is the current robot pose?"
- "Get all joint states"
- "Stop the robot"

See [rosa_agents/robot_URDF_agent/README_ROBOT.md](rosa_agents/robot_URDF_agent/README_ROBOT.md) for detailed documentation.

## 🔧 Customization

### Change Robot Configuration

Edit `rosa_agents/robot_URDF_agent/run_robot.py`:
```python
# Customize robot name and namespace
self.rosa = ROSA(
    ros_version=2,
    llm=llm,
    tools=robot_tools,
    prompts=RobotSystemPrompts(
        embodiment_and_persona="You are controlling a mobile robot named 'my_robot'.",
        about_your_capabilities="You can move, rotate, read joint states, and check position."
    )
)
```

### Change LLM Provider

Edit `rosa_agents/azure_setup.py` to use different LLM:
```python
# For OpenAI instead of Azure
from langchain_openai import ChatOpenAI
llm = ChatOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
```


## 📦 Project Structure

```
rosa_ws/
├── rosa_agents/                 # AI agent implementations
│   ├── turtlesim_agent/        # TurtleSim natural language control
│   │   ├── tool.py             # ROS2 turtlesim tools
│   │   ├── run_rosa.py         # Main agent script
│   │   └── __init__.py
│   ├── robot_URDF_agent/       # Generic URDF robot control
│   │   ├── robot_tools.py      # ROS2 robot control tools
│   │   ├── run_robot.py        # Main agent script
│   │   └── __init__.py
│   ├── azure_setup.py          # LLM configuration (shared)
│   ├── .env.example            # Environment variable template
│   ├── requirements.txt        # Python dependencies
│   └── README.md               # Agent-specific documentation
│
├── robot_files/                # Robot URDF packages
│   ├── my_robot_description/   # Robot models and configuration
│   └── my_robot_bringup/       # Launch files
│
├── rosa_venv/                  # Python virtual environment
├── .gitignore                  # Git ignore rules
└── README.md                   # This file
```

## 📦 Dependencies

Core Python packages (in `rosa_agents/requirements.txt`):
- `rclpy` - ROS2 Python client library
- `langchain` - LLM framework
- `langchain-openai` - OpenAI/Azure integration
- `rosa` - Robot Operating System Agent
- `python-dotenv` - Environment variable management

ROS2 packages:
- `turtlesim` - For turtlesim agent
- Your robot's packages - For robot agent (in `robot_files/`)



## 🐛 Troubleshooting

### Import errors
```bash
# Make sure you're in the venv
source rosa_venv/bin/activate

# Reinstall dependencies
cd rosa_agents
pip install -r requirements.txt
```

### ROS2 topics not found
```bash
# Check topics exist
ros2 topic list

# Verify robot is running
ros2 node list
```

### API key errors
```bash
# Verify .env file exists in rosa_agents/ and has correct keys
cat rosa_agents/.env

# Check environment variables are loaded
cd rosa_agents
python3 -c "import os; from dotenv import load_dotenv; load_dotenv(); print(os.getenv('AZURE_OPENAI_API_KEY'))"
```

### Running from wrong directory
```bash
# Agent scripts must be run from their respective directories
cd rosa_ws/rosa_agents/turtlesim_agent  # For turtlesim
# OR
cd rosa_ws/rosa_agents/robot_URDF_agent  # For URDF robot
```

## 📚 Resources

- [ROSA GitHub](https://github.com/nasa-jpl/rosa)
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [LangChain Documentation](https://python.langchain.com/)

## ⚠️ Important Notes
- Both agents must be run from their respective directories for imports to work correctly
- Make sure ROS2 is sourced before running agents: `source /opt/ros/jazzy/setup.bash`

**Built with ROSA for ROS2 Jazzy** 🤖✨
