import matplotlib.pyplot as plt

def plot_values():
    x_values = []
    y_values = []

    with open('path.txt', 'r') as f:
        for line in f:
            x, y = line.strip().split(',')
            x_values.append((x))
            y_values.append((y))

    plt.scatter(x_values, y_values)
    plt.xlabel('X values')
    plt.ylabel('Y values')
    plt.title('Generated X and Y values')
    plt.show()

plot_values()
