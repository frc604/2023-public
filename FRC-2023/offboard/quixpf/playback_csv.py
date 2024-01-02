import argparse
import csv
import time

from plotter import Plotter


def run(filename):
    with open(filename) as logfile:
        plotter = Plotter(0)
        plotter_queue, _ = plotter.start()
        time.sleep(5.0)  # Wait for plotter to load

        reader = csv.reader(logfile)
        for t, x, y, theta, has_vision in reader:
            estimate = (float(x), float(y), float(theta))
            plotter_queue.put((None, None, estimate, has_vision == "True"))
            time.sleep(0.01)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile", help="Path to the log file")
    args = parser.parse_args()
    run(args.logfile)
