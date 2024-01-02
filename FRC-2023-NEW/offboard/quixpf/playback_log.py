import argparse
import time
import pickle

from particle_filter import ParticleFilter
from plotter3d import Plotter3d


def run(filename, speed):
    with open(filename, "rb") as logfile:
        setup = pickle.load(logfile)
        if type(setup) == tuple and len(setup) == 4:
            (
                num_particles,
                init_odometry,
                retroreflective_targets,
                apriltag_targets,
            ) = setup
        else:
            # Handle old logs
            with open("./offboard/quixpf/logs/targets.log", "rb") as targetsfile:
                (
                    num_particles,
                    _,
                    retroreflective_targets,
                    apriltag_targets,
                ) = pickle.load(targetsfile)
            init_odometry = setup

        plotter = Plotter3d(num_particles)
        plotter_queue, plotter_targets_queue = plotter.start()
        plotter_targets_queue.put((retroreflective_targets, apriltag_targets))
        time.sleep(5.0)  # Wait for plotter to load

        pf = ParticleFilter(
            num_particles, init_odometry, retroreflective_targets, apriltag_targets
        )

        rows = []
        while True:
            try:
                rows.append(pickle.load(logfile))
            except EOFError:
                break

        log_start_time = rows[0][0]
        start_time = time.perf_counter()
        for t, odometry, vision, log_best_estimate, _ in rows:
            if speed is not None:
                log_elapsed_time = t - log_start_time
                print(f"Elapsed time: {log_elapsed_time}")
                while time.perf_counter() - start_time < log_elapsed_time / speed:
                    pass

            pf.predict(odometry)
            has_vision = pf.update(vision)
            best_estimate = pf.get_best_estimate()
            plotter_queue.put((vision, pf.particles, best_estimate, has_vision))
            if log_best_estimate != best_estimate:
                dx = log_best_estimate[0] - best_estimate[0]
                dy = log_best_estimate[1] - best_estimate[1]
                dtheta = log_best_estimate[2] - best_estimate[2]
                error = dx, dy, dtheta
                print(f"Not reproducing log! Error: {error}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile", help="Path to the log file")
    parser.add_argument("-s", type=float, help="Playback speed multiplier")
    args = parser.parse_args()
    run(args.logfile, args.s)
