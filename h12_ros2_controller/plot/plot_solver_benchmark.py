import os
import numpy as np
import matplotlib.pyplot as plt

def plot_benchmark(solver, filepath):
    data = np.load(f'data/{filepath}/{solver}.npz')
    linear_error_log = data['linear_error_log']  # shape: (N, T)
    angular_error_log = data['angular_error_log']  # shape: (N, T)

    # make dir
    os.makedirs(f'figures/{filepath}', exist_ok=True)

    # plot linear error
    plt.figure()
    for i in range(linear_error_log.shape[0]):
        plt.plot(linear_error_log[i], label=f'Target {i+1}')
    plt.title(f'Linear Error - {solver}')
    plt.xlabel('Time Step')
    plt.ylabel('Linear Error')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'figures/{filepath}/{solver}_linear_error.png')
    plt.close()

    # plot angular error
    plt.figure()
    for i in range(angular_error_log.shape[0]):
        plt.plot(angular_error_log[i], label=f'Target {i+1}')
    plt.title(f'Angular Error - {solver}')
    plt.xlabel('Time Step')
    plt.ylabel('Angular Error')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'figures/{filepath}/{solver}_angular_error.png')
    plt.close()

if __name__ == '__main__':
    solvers = ['osqp', 'proxqp', 'daqp', 'quadprog']
    filepath = 'compare_solver_onrobot'

    for solver in solvers:
        plot_benchmark(solver, filepath)
