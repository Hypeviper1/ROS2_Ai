# ROSA ROS2 Agents

Natural language control for ROS2 robots using ROSA (Robot Operating System Agent).

Includes two ready-to-use agents:
- **TurtleSim Agent** - Control turtlesim with natural language
- **URDF Robot Agent** - Control any URDF robot (Gazebo, RViz, or real hardware)

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 or later
- ROS2 Jazzy (or Humble/Iron)
- Python 3.10+
- Azure OpenAI API key (or OpenAI API key)

### Installation

```bash
# 1. Clone the repository
git clone <your-repo-url>
cd rosa_ws

# 2. Install ROS2 dependencies
sudo apt update
sudo apt install ros-jazzy-turtlesim  # For turtlesim agent

# 3. Create Python virtual environment
python3 -m venv rosa_venv
source rosa_venv/bin/activate

# 4. Install Python dependencies
pip install --upgrade pip
pip install rclpy langchain langchain-core langchain-openai langchain-community python-dotenv

# 5. Install ROSA (if not in requirements.txt)
pip install rosa  # Or install from source

# 6. Configure API keys
cp .env.example .env
nano .env  # Add your API keys
```

### Configure Environment Variables

Create a `.env` file in the root directory:

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
cd rosa_ws
source /opt/ros/jazzy/setup.bash
source rosa_venv/bin/activate
cd turtlesim_agent
python3 run_rosa.py
```

**Example commands:**
- "Move forward 2 meters"
- "Draw a square with 2 unit sides"
- "Rotate 90 degrees left"
- "Spawn a new turtle at (5, 5)"

## 🤖 Running Robot URDF Agent

```bash
# Terminal 1: Launch your robot (Gazebo/RViz/Hardware)
source /opt/ros/jazzy/setup.bash
source ~/your_robot_ws/install/setup.bash
ros2 launch <your_robot_pkg> <launch_file>.py

# Terminal 2: Run agent
cd rosa_ws
source /opt/ros/jazzy/setup.bash
source rosa_venv/bin/activate
cd robot_URDF_agent
python3 run_robot.py
```

**Example commands:**
- "Move forward 1 meter"
- "Rotate 45 degrees right"
- "What is the current robot pose?"
- "Get all joint states"
- "Stop the robot"

See [robot_URDF_agent/README_ROBOT.md](robot_URDF_agent/README_ROBOT.md) for detailed documentation.

## 🔧 Customization

### For Your Own Robot

1. Edit `robot_URDF_agent/run_robot.py`:
   ```python
   ROBOT_NAME = "your_robot_name"
   ROBOT_DESCRIPTION = """
   Your robot description here
   """
   ```

2. Adjust topic names in `robot_URDF_agent/robot_tools.py` if needed

3. Add custom tools for your robot's specific capabilities

### Change LLM Provider

Edit `azure_setup.py` to use different LLM:
```python
# For OpenAI instead of Azure
from langchain_openai import ChatOpenAI
llm = ChatOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
```

## 📦 Dependencies

Core Python packages:
- `rclpy` - ROS2 Python client library
- `langchain` - LLM framework
- `langchain-openai` - OpenAI/Azure integration
- `rosa` - Robot Operating System Agent
- `python-dotenv` - Environment variable management

ROS2 packages:
- `turtlesim` - For turtlesim agent
- Your robot's packages - For robot agent

## 🐛 Troubleshooting

### Import errors
```bash
# Make sure you're in the venv
source rosa_venv/bin/activate

# Reinstall dependencies
pip install --upgrade rclpy langchain langchain-openai
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
# Verify .env file exists and has correct keys
cat .env

# Check environment variables are loaded
python3 -c "import os; from dotenv import load_dotenv; load_dotenv(); print(os.getenv('AZURE_OPENAI_API_KEY'))"
```


## 📚 Resources

- [ROSA GitHub](https://github.com/nasa-jpl/rosa)
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/)
- [LangChain Documentation](https://python.langchain.com/)

## ⚠️ Important Notes

- **Never commit `.env` file** - It contains your API keys

**Built with ROSA for ROS2 Jazzy** 🤖✨
