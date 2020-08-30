""" graph.py
​
    Author:     Maddie Mooney <mxm7784@rit.edu>
    Date:       04/03/2020
    Modified:
        03/29/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -
"""

import matplotlib.pyplot as plt

plt.rcdefaults()

import numpy as np
import multiprocessing


class Graph:

    def __init__(self, labels=(), xmin=0, xmax=100, ymin=0, ymax=100, title='', hbar=False):
        """ Initialize graph.

            Args:
                labels  - Tuple of data labels. Default empty.
                xmin    - Lower bound of x axis. Default 0
                xmax    - Upper bound of x axis. Default 100.
                ymin    - Lower bound of y axis. Default 0.
                ymax    - Upper bound of y axis. Default 100.
                title   - Title of the plot. Default empty.
                hbar    - Is plot type hbar? Default False.
        """

        # Initialize queue & queue proc
        self.data_queue = multiprocessing.Queue()
        self.queue_proc = multiprocessing.Process(
            target=self.draw_plot,
            args=(self.data_queue, labels, xmin, xmax, ymin, ymax, title, hbar)
        )
        self.queue_proc.start()

    def __del__(self):
        plt.close()
        self.queue_proc.join()

    def add(self, data):
        self.data_queue.put(data)

    @staticmethod
    def draw_plot(queue, labels=(), xmin=0, xmax=100, ymin=0, ymax=100, title='', hbar=False):
        """ Refreshes the plot.

            Args
                queue   - Data to be plotted
        """

        # Lock axis range
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)

        if labels:
            objects = labels
        else:
            objects = tuple(range(0, 19))  # TODO: How to pass in different values?

        # Evenly space the bars according to how many there are
        x_pos = np.arange(len(objects))
        # bars of bar graph...
        data_to_plot = range(0, len(objects))

        # What and where to label the graph
        plt.xticks(x_pos, objects)

        # Add title to plot
        plt.title(title)

        # Turn interactive mode on. Allows us to refresh w/out making new graph
        plt.ion()  # put in constructor?
        # Fetch the figure
        fig = plt.figure()

        while True:
            # Data will come in from queue faster than disp. So clear queue each time
            while not queue.empty():
                # Fetch item from the queue and store it locally.
                data_to_plot = queue.get()

            # Clear the axis
            plt.cla()

            # Add title to plot
            plt.title(title)

            # Make the plots
            if hbar:
                plt.barh(x_pos, data_to_plot, align='center')
                # What and where to label the graph
                plt.xlim(xmin, xmax)
                plt.yticks(x_pos, objects)
            else:
                plt.bar(x_pos, data_to_plot, align='center')
                plt.ylim(ymin, ymax)
                # What and where to label the graph
                plt.xticks(x_pos, objects)

            # Draw the new graph
            fig.canvas.draw()

            # Flush the GUI events for the figure
            fig.canvas.flush_events()
