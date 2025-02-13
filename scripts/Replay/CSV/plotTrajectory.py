
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pandas

# Plot the trajectory data from the CSV file named by the user

filename = input("\nEnter the path or name of the CSV file: ")

# Load the data from the CSV file using pandas
data = pandas.read_csv(filename)

# Create a figure and 3D axes
figure = plt.figure()
ax = plt.axes(projection='3d')

# Extract the X, Y, and Z values from the data
x = data['x']
y = data['y']
z = data['z']

# Plot the data
ax.scatter(x, y, z)

# Set the labels for the axes
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')

# Show the plot
plt.show()
