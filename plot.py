import matplotlib.pyplot as plt
import numpy as np
import csv
import os
import logging

class Evaluator:
    @staticmethod
    def load_data(file_path):
        data = {}
        try:
            with open(file_path, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip header
                for row in reader:
                    if len(row) != 7:
                        logging.warning(f"Row length mismatch in file {file_path}")
                        continue
                    timestamp = float(row[0])
                    values = list(map(float, row[1:]))
                    data[timestamp] = values
        except FileNotFoundError:
            logging.error(f"File not found: {file_path}")
            return None
        except Exception as e:
            logging.error(f"Error reading file {file_path}: {e}")
            return None

        return data

    @staticmethod
    def angle_difference(angle1, angle2):
        """ Calculate the smallest difference between two angles, considering wrapping. """
        difference = angle1 - angle2
        difference = (difference + 180) % 360 - 180  # Wrap to [-180, 180]
        return difference

    @staticmethod
    def calculate_statistics(source_file, reference_file):
        source_data = Evaluator.load_data(source_file)
        reference_data = Evaluator.load_data(reference_file)

        if source_data is None or reference_data is None:
            return None

        source_times = np.array(list(source_data.keys()))
        reference_times = np.array(list(reference_data.keys()))

        if len(reference_times) == 0 or len(source_times) == 0:
            logging.warning(f"No timestamps available for calculation.")
            return None

        labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        statistics = {}

        for i in range(6):  # Iterate over X, Y, Z, Roll, Pitch, Yaw
            ref_values = np.array([reference_data[t][i] for t in reference_times])
            interpolated_values = np.interp(source_times, reference_times, ref_values)
            source_values = np.array([source_data[t][i] for t in source_times])

            if labels[i] in ['Roll', 'Pitch', 'Yaw']:
                # Handle angle wrapping
                errors = np.array([Evaluator.angle_difference(src, ref) for src, ref in zip(source_values, interpolated_values)])
            else:
                # Normal subtraction for X, Y, Z
                errors = source_values - interpolated_values

            statistics[labels[i]] = {
                "Mean Error": np.mean(errors),
                "STDDEV Error": np.std(errors),
                "RMSE": np.sqrt(np.mean(errors ** 2))
            }

        return statistics

class Plotter:
    COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']

    @staticmethod
    def plot_time_series(file_paths, file_labels):
        fig, axs = plt.subplots(6, 1, sharex=True, figsize=(10, 10))
        labels = ['X Position', 'Y Position', 'Z Position', 'Roll', 'Pitch', 'Yaw']
        fig.suptitle('State estimates and motion capture over time')
        print("Plotting time series...")
        for idx, file_path in enumerate(file_paths):
            label = file_labels[idx] if idx < len(file_labels) else f"File {idx + 1}"
            color = Plotter.COLORS[idx % len(Plotter.COLORS)]
            data = Evaluator.load_data(file_path)
            if data:
                timestamps = np.array(list(data.keys()))
                values = np.array(list(data.values()))

                for label_index, label_name in enumerate(labels):
                    axs[label_index].plot(timestamps, values[:, label_index], label=label, color=color)

        for i, ax in enumerate(axs):
            ax.grid(True, which='both', linestyle='--', alpha=0.7)
            ax.legend()
            ax.set_ylabel(labels[i])

        axs[-1].set_xlabel('Time (ms)')
        plt.tight_layout()
        return fig

    @staticmethod
    def plot_error_scatter(statistics_list, labels):
        position_metrics = ['X', 'Y', 'Z']
        angle_metrics = ['Roll', 'Pitch', 'Yaw']
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('Mean error of state estimates with respect to motion capture')

        for metrics, ax, title in zip([position_metrics, angle_metrics], [ax1, ax2], ['Position Error', 'Angle Error']):
            x = np.arange(len(metrics))

            for i, (stats, label) in enumerate(zip(statistics_list, labels)):
                if stats is None:
                    continue
                means = np.array([stats[metric]["Mean Error"] for metric in metrics])
                errors = np.array([stats[metric]["STDDEV Error"] for metric in metrics])
                ax.errorbar(x + i * 0.1, means, yerr=errors, fmt='o', label=label, color=Plotter.COLORS[i % len(Plotter.COLORS)], capsize=5)

            ax.set_xticks(x)
            ax.set_xticklabels(metrics)
            ax.grid(True, axis='y', linestyle='--', alpha=0.7)
            ax.set_ylabel('Mean Error')
            ax.legend()

        plt.tight_layout()
        return fig

    @staticmethod
    def plot_rmse_comparison(statistics_list, labels):
        position_metrics = ['X', 'Y', 'Z']
        angle_metrics = ['Roll', 'Pitch', 'Yaw']
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('RMSE of state estimates with respect to motion capture')

        for metrics, ax, title in zip([position_metrics, angle_metrics], [ax1, ax2], ['Position RMSE', 'Orientation RMSE']):
            rmse_values = [np.array([stats[metric]['RMSE'] for metric in metrics]) for stats in statistics_list if stats is not None]
            x = np.arange(len(metrics))
            width = 0.35

            for i, (rmse, label) in enumerate(zip(rmse_values, labels)):
                ax.bar(x + i * width, rmse, width, label=label, color=Plotter.COLORS[i % len(Plotter.COLORS)])

            ax.set_xticks(x + width / 2)
            ax.set_xticklabels(metrics)
            ax.grid(True, axis='y', linestyle='--', alpha=0.7)
            ax.set_ylabel('RMSE')
            ax.legend()

        plt.tight_layout()
        return fig

    @staticmethod
    def plot_error_time_series(file_paths, labels):
        fig, axs = plt.subplots(6, 1, sharex=True, figsize=(10, 12))
        metrics = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        fig.suptitle('Error Time Series wrt Mocap')

        mocap_data = Evaluator.load_data(file_paths[2])
        mocap_times = np.array(list(mocap_data.keys()))

        for idx, source_file in enumerate(file_paths[:2]):
            source_data = Evaluator.load_data(source_file)
            source_times = np.array(list(source_data.keys()))

            for i, metric in enumerate(metrics):
                ref_values = np.array([mocap_data[t][i] for t in mocap_times])
                interpolated_values = np.interp(source_times, mocap_times, ref_values)
                source_values = np.array([source_data[t][i] for t in source_times])

                if metric in ['Roll', 'Pitch', 'Yaw']:
                    errors = np.array([Evaluator.angle_difference(src, ref) for src, ref in zip(source_values, interpolated_values)])
                else:
                    errors = source_values - interpolated_values

                axs[i].plot(source_times, errors, label=labels[idx], color=Plotter.COLORS[idx])

        for i, ax in enumerate(axs):
            ax.grid(True, linestyle='--', alpha=0.7)
            ax.set_ylabel(f'{metrics[i]} Error')
            ax.legend()

        axs[-1].set_xlabel('Time (ms)')
        plt.tight_layout()
        return fig

if __name__ == "__main__":
    file_paths = [
        'output/legacy_yaw_c.csv',
        'output/full_kalman_c.csv',
        'output/mocap_c.csv'
    ]
    file_labels = ['Legacy', 'Kalman', 'Mocap']

    statistics_list = [
        Evaluator.calculate_statistics(file_paths[0], file_paths[2]),
        Evaluator.calculate_statistics(file_paths[1], file_paths[2])
    ]

    # Collect figures in a list:
    figs = []

    fig_scatter = Plotter.plot_error_scatter(statistics_list, file_labels[:-1])
    figs.append(fig_scatter)

    fig_rmse = Plotter.plot_rmse_comparison(statistics_list, file_labels[:-1])
    figs.append(fig_rmse)

    fig_time = Plotter.plot_time_series(file_paths, file_labels)
    figs.append(fig_time)

    fig_err = Plotter.plot_error_time_series(file_paths, file_labels[:-1])
    figs.append(fig_err)

    # Now show them all at once:
    for fig in figs:
        fig.show()

    # The following ensures the event loop doesn't close:
    input("Press Enter to close plots...")
