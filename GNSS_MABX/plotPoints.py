import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a pandas DataFrame with column names "lat" and "long"
data = pd.read_csv('waypoints.csv', names=["lat", "long"])

# Scatter plot the lat-long points
plt.scatter(data["long"], data["lat"], s=10)

# Set plot title and axis labels
plt.title("Lat-Long Scatter Plot")
plt.xlabel("Longitude")
plt.ylabel("Latitude")

# Show the plot
plt.show()
