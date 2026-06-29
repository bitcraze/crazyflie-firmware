import argparse

import cfusdlog
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Plot uSD log data from Crazyflie")
    parser.add_argument("filename", help="Path to the binary log file")
    args = parser.parse_args()

    data = cfusdlog.decode(args.filename)

    for event_name, log_data in data.items():
        timestamp = log_data["timestamp"]
        variables = sorted(k for k in log_data if k != "timestamp")

        if not variables:
            print(f"No variables found in event '{event_name}', skipping.")
            continue

        fig, axes = plt.subplots(len(variables), 1, sharex=True, squeeze=False)
        fig.suptitle(event_name)

        for ax, key in zip(axes.flat, variables):
            ax.plot(timestamp, log_data[key])
            ax.set_ylabel(key)

        axes.flat[-1].set_xlabel("timestamp [ms]")
        fig.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
