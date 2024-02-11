import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np
np.float = float 

def extract_scan_values(filename):
    scan_values_list = []
    with open(filename, 'r') as file:
        for line in file:
            # Split the line by whitespace (assuming values are separated by whitespace)
            scan_values = line.strip().split(',')
            # Convert the scan values to floats, replace 'inf' with 0, and store them as a list
            scan_values = [3.5 if (value == 'inf' or value == ' inf') else float(value) for value in scan_values]
            scan_values_list.append(scan_values)
    return scan_values_list


# Function to plot scan values in a radial plot
def plot_radial_scan_values(scan_values):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    plt.subplots_adjust(bottom=0.25)
    plt.title("Radial Scan Values")

    # Plot the initial scan
    initial_scan_id = 0
    # scan_id = 0
    theta = np.linspace(0, 2 * np.pi, len(scan_values[initial_scan_id]), endpoint=False)
    line, = ax.plot(theta, scan_values[initial_scan_id])

    # Create slider
    ax_scan_id = plt.axes([0.1, 0.1, 0.65, 0.03])
    slider = Slider(ax_scan_id, 'Scan ID', 0, len(scan_values) - 1, valinit=initial_scan_id)

    # Update function for the slider
    def update(val):
        scan_id = int(slider.val)
        theta = np.linspace(0, 2 * np.pi, len(scan_values[scan_id]), endpoint=False)
        scan = scan_values[scan_id]
        line.set_ydata(scan)
        line.set_xdata(theta)
        fig.canvas.draw_idle()

    slider.on_changed(update)

    plt.show()
    
    
if __name__ == "__main__":
    filename = "scan_clean_data.txt"
    scan_values = extract_scan_values(filename)
    # print("Scan Values:")
    # for idx, scan in enumerate(scan_values, 1):
    #     print(f"Scan {idx} size{len(scan)}")
    plot_radial_scan_values(scan_values)