import numpy as np
import matplotlib.pyplot as plt

# Read the list of tuples from the .txt file
def read_data(filename):
    with open(filename, 'r') as file:
        data = []
        for line in file:
            x, y = line.strip()[1:-1].split(',')
            data.append((float(x), float(y)))
        return data

# Fit a polynomial regression to the data
def fit_polynomial(data, degree):
    x_data = [point[0] for point in data]
    y_data = [point[1] for point in data]
    coeffs = np.polyfit(x_data, y_data, degree)
    return np.poly1d(coeffs)

# Plot the data points and fitted graph
def plot_data(data, ):
    x_data = [point[0] for point in data]
    y_data = [point[1] for point in data]
    plt.scatter(x_data, y_data, label='Data Points')
    
    #x_fit = np.linspace(min(x_data), max(x_data), 100)
    #y_fit = fitted_curve(x_fit)
    #plt.plot(x_fit, y_fit, 'r', label='Fitted Curve')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Data Plot with Fitted Curve')
    plt.legend()
    plt.grid(True)
    plt.show()

# Main program
if __name__ == '__main__':
    filename = '/home/peter/ros2_ws/src/lidar_reg/lidar_reg/ResponseGraphLINVELTIMENEWActual.txt'  # Replace with your .txt file path
    degree = 2  # Degree of polynomial regression
    
    data = read_data(filename)
    plot_data(data)
