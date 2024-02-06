# -*- coding: utf-8 -*-
"""
plotting a generic USD log
"""
import cfusdlog
import matplotlib.pyplot as plt
import argparse
import numpy as np
import mplcursors
import functools

def showAnnotation(data, sel):
    idx = sel.target.index
    sel.annotation.set_text(
        "\n".join(['{}: {}'.format(key, data[key][idx]) for key in data.keys()]))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    args = parser.parse_args()

    # decode binary log data
    data_usd = cfusdlog.decode(args.file_usd)

    # find start time
    start_time = None
    for k, (event_name, data) in enumerate(data_usd.items()):
        if start_time is None:
            start_time = data['timestamp'][0]
        else:
            start_time = min(start_time, data['timestamp'][0])

    # new figure
    fig, ax = plt.subplots(len(data_usd.keys()),1,sharex=True,squeeze=False)

    for k, (event_name, data) in enumerate(data_usd.items()):
        # print(k, event_name)
        t = (data['timestamp'] - start_time) / 1000
        ax[k,0].scatter(t, t*0)
        ax[k,0].set_title(event_name)

        print(data.keys())

        crs = mplcursors.cursor(ax[k],hover=True)

        crs.connect("add", functools.partial(showAnnotation, data))

    ax[-1,0].set_xlabel('Time [s]')


    plt.show()
