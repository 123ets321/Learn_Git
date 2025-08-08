# ROS 2 Ghost Workspace

A ROS 2 workspace configured for PX4 simulation and ROS 2 integration using **Micro XRCE-DDS Agent**. Develop, test, and simulate PX4-based aerial vehicles with ROS 2 packages.

## 📂 Workspace Structure

```
ros2-ghost-ws/
├── PX4-Autopilot/    # PX4 firmware source
├── src/              # ROS 2 packages
└── README.md
```

## 🔧 Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** - [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- `colcon` build tool
- `git`, `python3`, `cmake`, and build essentials
- PX4 simulation dependencies - [PX4 Dev Guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)

## 🛠️ Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/theskytex/ros2-ghost-ws.git
   cd ros2-ghost-ws
   ```

2. **Build ROS 2 packages:**
   ```bash
   colcon build
   ```

3. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## 🚀 Running the Simulation

### Start PX4 SITL
```bash
cd PX4-Autopilot
make px4_sitl gz_standard_vtol
```

### Start Micro XRCE-DDS Agent
In a new terminal:
```bash
MicroXRCEAgent udp4 -p 8888
```
*Bridges PX4 uORB messages to ROS 2 topics*

### Run ROS 2 Nodes
In another terminal:
```bash
source install/setup.bash
ros2 run <your_package> <your_node>
```

## 🖥️ Multi-Terminal Workflow

| Terminal | Command | Purpose |
|----------|---------|---------|
| 1 | `cd PX4-Autopilot && make px4_sitl gz_standard_vtol` | PX4 simulation |
| 2 | `MicroXRCEAgent udp4 -p 8888` | Message bridge |
| 3 | `source install/setup.bash && ros2 run vtol_stack vtol_comms` | ROS 2 communications |
| 4 | `source install/setup.bash && ros2 run vtol_stack dive_vtol` | ROS 2 control |

## 📚 References

- [PX4 Documentation](https://docs.px4.io/v1.15/en/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
