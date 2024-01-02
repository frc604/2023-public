from multiprocessing import Process
import os

from run_simple_test import plan as plan_simple
from run_square_test import plan as plan_square
from run_bottom_cone_cube_cube_score import plan as plan_bottom_cone_cube_cube_score
from run_top_cone_cube_cube_score import plan as plan_top_cone_cube_cube_score
from run_bottom_cone_cube_charge import plan as plan_bottom_cone_cube_charge
from run_top_cone_cube_charge import plan as plan_top_cone_cube_charge
from run_bottom_cube_cube_cube_score import plan as plan_bottom_cube_cube_cube_score
from run_top_cube_cube_cube_score import plan as plan_top_cube_cube_cube_score
from run_mid_cone import plan as plan_mid_cone
from run_mid_cone_charge import plan as plan_mid_cone_charge


if __name__ == "__main__":
    # Delete old plots and CSVs
    dirname = os.path.dirname(__file__)
    for item in os.listdir(os.path.join(dirname, "plots")):
        if item.endswith(".png"):
            os.remove(os.path.join(dirname, "plots", item))
    for item in os.listdir(os.path.join(dirname, "../../src/main/deploy")):
        if item.endswith(".csv"):
            os.remove(os.path.join(dirname, "../../src/main/deploy", item))

    # Run paths without alliance here
    Process(target=plan_simple, args=(True,)).start()
    Process(target=plan_square, args=(True,)).start()
    for allilance_color in ["blue", "red"]:
        # Run paths with alliance here
        Process(
            target=plan_bottom_cone_cube_cube_score, args=(allilance_color, True)
        ).start()
        Process(
            target=plan_top_cone_cube_cube_score, args=(allilance_color, True)
        ).start()
        Process(
            target=plan_bottom_cone_cube_charge, args=(allilance_color, True)
        ).start()
        Process(target=plan_top_cone_cube_charge, args=(allilance_color, True)).start()
        Process(
            target=plan_bottom_cube_cube_cube_score, args=(allilance_color, True)
        ).start()
        Process(
            target=plan_top_cube_cube_cube_score, args=(allilance_color, True)
        ).start()
        Process(target=plan_mid_cone, args=(allilance_color, True)).start()
        Process(target=plan_mid_cone_charge, args=(allilance_color, True)).start()
