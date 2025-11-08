# ROS2-Turtlesim
This ROS2 package demonstrates drawing various geometric shapes using the **turtlesim** simulator.   It also provides examples of controlling turtlesim through different **ROS2 services** like spawning new turtles, clearing the screen, teleporting, and killing turtles.

This package is ideal for beginners learning ROS2 concepts such as **nodes**, **topics**, **services**, and **geometry control** with turtlesim.

---

## ✨ Features

### 🧩 Shape Drawing Nodes
Implements automatic drawing of different geometric shapes in **turtlesim**:

| Shape Name | Description |
|-------------|--------------|
| 🟢 Circle | Turtle moves in a perfect circular path using angular velocity control |
| 🔷 Square | Turtle moves in a four-sided square pattern |
| 🔺 Triangle | Turtle moves in an equilateral triangle |
| ⭐ Star | Turtle draws a classic five-pointed star shape |
| 🔸 Hexagon | Turtle draws a six-sided regular hexagon |
| 🟣 Octagon | Turtle draws an eight-sided regular octagon |
| 🌟 12-Star | Turtle draws a 12-pointed complex star shape |

Each shape node uses **velocity commands (`/turtle1/cmd_vel`)** and geometry calculations to draw accurate shapes in the simulator.

---

## 🧰 Turtlesim Service Integrations

This package also demonstrates the use of **turtlesim services**:

| Service Name | Description |
|---------------|-------------|
| `/clear` | Clears all drawings on the turtlesim canvas |
| `/kill` | Removes an existing turtle by name |
| `/spawn` | Spawns a new turtle at a given `(x, y, θ)` position |
| `/teleport_absolute` | Instantly teleports the turtle to a specific location |
| `/teleport_relative` | Moves the turtle relative to its current position |

These services are implemented as dedicated Python or C++ nodes for easy understanding and testing.

---

## 🧠 Learning Objectives

By using this package, you will learn:

- How to create **ROS2 nodes** for motion control  
- How to use **Publishers** (`/cmd_vel`) and **Services** (`/spawn`, `/kill`, etc.)  
- How to perform **coordinate transformations** for shape drawing  
- How to manage **multiple turtles** in a simulation  
- How to integrate geometry and math concepts in robot motion  

---

## 🏗️ Package Structure

```
├── shapes_pkg/
  ├── setup.py
  ├── setup.cfg
  ├── package.xml
  ├── simple_pkg_python/
  │ ├── init.py
  │ ├── circle.py                     # circle shape node
  │ ├── clear.py                      # clear service node
  │ ├── hexagon.py                    # hexagon shape node
    .
    .
    .                                 # etc, all types of shapes and service nodes
  ├── resource/
  │ └── simple_pkg_python
  ├── launch/
  │ ├── turtle.launch.py              # launch file
  │ └── game.launch.py
```

---

## 🚀 How to Run

### Source ROS2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### Clone Repository
```bash
git clone https://github.com/yashbhaskar/ROS2-Turtlesim.git
```

### Build the Package
```bash
colcon build --packages-select shapes_pkg
source install/setup.bash
```

### Launch Turtlesim
```bash
ros2 run turtlesim turtlesim_node
```

### Run Node (Shape/Service)
```bash
ros2 run shapes_pkg {change this with node name}
```

---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com

📌 GitHub: https://github.com/yashbhaskar



