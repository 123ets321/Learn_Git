# ros2-ghost-ws

---

```markdown
# ROS 2 Ghost Workspace

This repository contains a ROS 2 workspace configured for PX4 simulation and ROS 2 integration using **Micro XRCE-DDS Agent**.  
It is intended for developing, testing, and simulating PX4-based aerial vehicles with ROS 2 packages in `src/`.

## 📂 Workspace Structure

```

ros2-ghost-ws/
├── PX4-Autopilot    # PX4 firmware source
├── src              # ROS 2 packages
└── README.md

````

---

## 🔧 Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** ([ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- `colcon` build tool
- `git`, `python3`, `cmake`, and build essentials
- PX4 simulation dependencies ([PX4 Dev Guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html))

---

## 🛠️ Build Instructions

1. **Clone the repository**
   ```bash
   git clone https://github.com/theskytex/ros2-ghost-ws.git
   cd ros2-ghost-ws
````

2. **Build ROS 2 packages**

   ```bash
   colcon build
   ```

3. **Source the workspace**

   ```bash
   source install/setup.bash
   ```

---

## 🚀 Running the Simulation

### 1️⃣ Start PX4 SITL

From the PX4 directory:

```bash
make px4_sitl gz_standard_vtol
```

---

### 2️⃣ Start Micro XRCE-DDS Agent

In a separate terminal:

```bash
MicroXRCEAgent udp4 -p 8888
```

This bridges PX4 uORB messages to ROS 2 topics.

---

### 3️⃣ Run Your ROS 2 Nodes

In another terminal (after sourcing your workspace):

```bash
source install/setup.bash
ros2 run <your_package> <your_node>
```

---

## 🖥️ Typical Workflow

1. **Terminal 1:** Run PX4 simulation

   ```bash
   cd PX4-Autopilot
   make px4_sitl gz_standard_vtol
   ```

2. **Terminal 2:** Run MicroXRCEAgent

   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

3. **Terminal 3:** Run ROS 2 node(s)

   ```bash
   source install/setup.bash
   ros2 run vtol_stack vtol_comms
   ```

4. **Terminal 4:** Run ROS 2 node(s)

   ```bash
   source install/setup.bash
   ros2 run vtol_stack dive_vtol
   ```




---

## 📚 References

* [PX4 Documentation](https://docs.px4.io/v1.15/en/)
* [ROS 2 Documentation](https://docs.ros.org/en/humble/)
* [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)

---
