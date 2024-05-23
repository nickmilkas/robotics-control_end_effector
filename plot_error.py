import numpy as np
import matplotlib.pyplot as plt


def plot_error_with_repetition(error_array_list):
    errors = [np.linalg.norm(arr) for arr in error_array_list]

    num_of_reps = len(errors)

    plt.plot(np.arange(1, num_of_reps + 1), errors)
    plt.title('Error in Simulation')
    plt.xlabel('Number of Repetitions')
    plt.ylabel('Error')
    plt.grid(True)
    plt.show()
