import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt

# Correct file path; update this according to your actual file location
file_path = './LE2 Simulation and Analysis/Injector Analysis/Data/PropCalcs.xlsx'

# Load the Excel file and display the available sheets to verify the correct one is being accessed
excel_sheets = pd.ExcelFile(file_path)
print(excel_sheets.sheet_names)  # This should show all sheet names; verify 'Copy of Flow Testing' is listed

# Load the specific sheet data
flow_testing_data = pd.read_excel(file_path, sheet_name='LOX Flow Testing')

# Extract the relevant columns and drop any rows with missing data
relevant_data = flow_testing_data[['Unnamed: 3', 'Unnamed: 12', 'Unnamed: 9']]
relevant_data.columns = ['Orifice Diameter (m)', 'Cd', 'Reynolds Number']
formatted_array = relevant_data.dropna().values  # Convert to numpy array, dropping rows with any NaN values and excluding string columns
# Remove the first row of data from each column
formatted_array = formatted_array[1:]

# Show the modified formatted array
print(formatted_array)
# Show the formatted array
print(formatted_array)
# Extract the relevant columns for the surface plot
x = formatted_array[:, 0].astype(float)  # Orifice Diameter (m)
y = formatted_array[:, 2].astype(float)  # Reynolds Number
z = formatted_array[:, 1].astype(float)  # Cd

# Apply Gaussian smoothing to the data
smoothed_z = gaussian_filter(z, sigma=3)

# Create a 3D figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create the surface plot with smoothed data
ax.plot_trisurf(x, y, smoothed_z, cmap='viridis')

# Set labels and title
ax.set_xlabel('Orifice Diameter (m)')
ax.set_ylabel('Reynolds Number')
ax.set_zlabel('Cd')
ax.set_title('Surface Plot of Cd, Gaussian Smoothing')

# Show the plot
plt.show()


