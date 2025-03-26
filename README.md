# Franka Emika Panda Operator Interface 

## Overview  
This repository provides tools for teleoperation, pick-and-place tasks, and trajectory recording/replay for the Franka Emika Panda manipulator.  

---

## Features  

###  **Teleoperation**  
- **Keyboard Control**: Directly control the Panda arm using keyboard inputs.  
- **Trajectory Replay**: Replay recorded end-effector trajectories.  

###  **Pick and Place**  
- **Object Manipulation**: Perform pick-and-place tasks using the MoveIt Python interface.  

---

## 📊 **Trajectory Tools**  

### **CSV-Based (Data Logging & Visualization)**  
| Script | Description |  
|--------|-------------|  
| `recordPoseCSV.py` | Records end-effector pose (XYZ + quaternion) into a CSV file. |  
| `recordTrajectory.py` | Captures full trajectory data (XYZ + quaternion) and saves as CSV. |  
| `plotTrajectory.py` | Plots trajectory from a specified CSV file for analysis. |  

### **Serialized (Binary Storage & Replay)**  
| Script | Description |  
|--------|-------------|  
| `recordPoseByte.py` | Saves end-effector pose (serialized binary) to a custom file. |  
| `goToPoseByte.py` | Executes movement to a pose stored in a binary file. |  

---

## Usage  
1. **Teleop**: Run keyboard control, then replay movements.  
2. **PickAndPlace**: Configure objects and run the MoveIt pipeline.  
3. **Trajectories**:  
   - Record → Plot (CSV)  
   - Record → Replay 

> **Note**: Ensure ROS and MoveIt are properly configured before use.  