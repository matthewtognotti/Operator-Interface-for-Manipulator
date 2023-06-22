# Franka Emika Panda 🐼

## Teleop

- Control the panda with the keyboard and replay trajectories in the end effector space

## PickAndPlace

- Pick and place objects using the Franka Emika Panda with the MoveIt Python interface. 


## Record

#### CSV (for plotting trajectories)

- recordPoseCSV.py records the end effector pose in xyz and quaternions stores the data in a CSV file named by the user

- recordTrajectory.py records the trajectory data of the end effector in xyz and quaternions and stores the data in a CSV file named by the user
  
- plotTrajectory.py plots the trajectory data from a CSV file named by the user


#### Serialized (for modifying trajectories, replaying trajectories, and saving poses)

- recordPoseByte.py records the end effector pose in xyz and quaternions, serializes the data, and stores the data in a file named by the user

- goToPoseByte.py tells the Panda to go to the pose saved in the file from recordPanda.py

