import os
import argparse
import numpy as np
import matplotlib.pyplot as plt

def plot_benchmark(solver, filepath, savepath):
    data = np.load(filepath)
    linear_error_log = data['linear_error_log']  # shape: (N, T)
    angular_error_log = data['angular_error_log']  # shape: (N, T)

    # make dir
    os.makedirs(savepath, exist_ok=True)

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
    plt.savefig(f'{savepath}/{solver}_linear_error.png')
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
    plt.savefig(f'{savepath}/{solver}_angular_error.png')
    plt.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot solver benchmark results')
    parser.add_argument('folder',
                        help='Folder name containing benchmark results in data/compare_solver/')
    args = parser.parse_args()

    solvers = ['osqp', 'proxqp', 'daqp', 'quadprog']

    for solver in solvers:
        filepath = f'./data/compare_solver/{args.folder}/{solver}.npz'
        savepath = f'./figures/compare_solver/{args.folder}'
        plot_benchmark(solver, filepath, savepath)
