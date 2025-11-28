# Auto-Waypoint-Navigation
Autonomous indoor exploration system for ROS2 using auto-generated waypoints. Automatically detects unexplored areas from the SLAM map, generates navigation goals, and drives the robot until the entire map is covered.

## 📌 Architecture Overview
```bash
SLAM → /map → Waypoint Generator → /next_waypoint → Nav2 → Robot moves → 
Map updates → Repeat
```

## 🧭 HIGH-LEVEL FLOW OF THE WHOLE EXPLORATION SYSTEM

```mermaid
flowchart TD

    A[Gazebo Simulation] 
        -->|Sensor data<br/>(LIDAR, Camera)| B[SLAM<br/>(slam_toolbox etc.)]

    B -->|Publishes map| C[/map topic<br/>(OccupancyGrid)]

    C --> D[Map Listener Node]

    D -->|Reads map data| E[Waypoint Generator Node<br/>- Scan map for unknown areas<br/>- Cluster unexplored zones<br/>- Compute waypoint centroid]

    E -->|Publishes next goal| F[/next_waypoint topic<br/>(PoseStamped)]

    F --> G[Waypoint Executor Node<br/>- Receives waypoint<br/>- Sends goal to Nav2]

    G -->|Nav2 Action Goal| H[Nav2 Navigation Stack<br/>- Global planner<br/>- Local planner<br/>- Controller + costmaps]

    H -->|Robot moves| I[Robot Moves]

    I -->|SLAM updates map| B
```

## 🧠 WHAT EACH COMPONENT DOES (HUMAN LANGUAGE EXPLANATION)

### 1️⃣ Gazebo

1. Simulates robot + sensors.
2. Publishes LIDAR data → goes into SLAM.

### 2️⃣ SLAM (Map Building)

SLAM converts raw sensor data into Occupancy Grid:
```bash
/map → OccupancyGrid (free, occupied, unknown)
```
SLAM updates the map continuously as the robot moves.

### 3️⃣ Map Listener Node

**Purpose:**
Just stores the latest map in a variable.

**Why needed?**
1. The waypoint generator must read the map.
2. But best practice is not to subscribe inside multiple nodes → so we separate it.

**Publisher/subscriber:**
1. Subscribes → /map
2. No publishers.

### 4️⃣ Waypoint Generator

This is the intelligent part.

It reads the map and decides where the robot should go next.

**Steps it performs:**
 
1. Convert occupancy grid into a usable array
2. Find all unexplored regions (-1 in map)
3. Cluster them into big logical zones
4. Pick 1 zone to explore
5. Compute the (x, y) world coordinate
6. Publish that as a waypoint

**Publisher/subscriber:**
1. Subscribes → /map
2. Publishes → /next_waypoint (PoseStamped)

### 5️⃣ Waypoint Executor

**Purpose:**
Take a waypoint and tell Nav2 to go there.

**Publisher/subscriber/action:**
1. Subscribes → /next_waypoint
2. Action Client → navigate_to_pose (Nav2)

**It waits until:**
1. Nav2 reaches the waypoint
2. Then waits for the next waypoint from the generator

### 6️⃣ Nav2

**This performs actual navigation:**
1. Global planner → long path
2. Local planner → avoid obstacles
3. Controller → movement
4. Costmaps → safe distances
5. Recovery behaviours

**It uses:**
1. /tf
2. /odom
3. /map
4. Laser scans

### 7️⃣ Robot Moves → SLAM updates → cycle repeats

**Every time the robot moves:**
1. SLAM updates the map
2. New unexplored areas appear
3. New waypoints get generated
4. Nav2 moves again.
Until the map is fully explored.

## 💡 HOW WE DECIDE WHICH MESSAGE TYPES TO USE
**A. What does the node need to read?**
🔹 Map listener needs a map → /map → OccupancyGrid
So it subscribes using:
```bash
ros2 topic info /map
ros2 interface show nav_msgs/msg/OccupancyGrid
```
```python
create_subscription(OccupancyGrid, '/map', ...)
```

**B. What does the next node need?**
1. Waypoint generator needs the map → so it subscribes to the same topic.
2. But it also outputs a waypoint → it should publish a PoseStamped because Nav2's NavigateToPose action expects: geometry_msgs/PoseStamped

**C. How do we send the waypoint to Nav2?**
Nav2 uses actions, not topics.
```bash
nav2_msgs/action/NavigateToPose
```
So the executor must be an Action Client:
```python
nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
```

## Notes
1. Install sklearn if not already
```bash
pip3 install scikit-learn
or
sudo apt install python3-sklearn
```

2. scikit-learn + SciPy DO NOT work with NumPy 2.x 
Use Pure NumPy coarse grid clustering instead of scikit-learn, sciPy and KMean
