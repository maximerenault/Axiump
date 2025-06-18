from OCC.Display.SimpleGui import init_display
from axiumplib import RotorBuilder, RotorParameters, FLAT, FilletSolidError
from axiumplib import save_shape_to_brep, save_shape_to_step
import numpy as np
from math import atan

MATPLOTLIB = False
try:
    import matplotlib.pyplot as plt

    MATPLOTLIB = True
except ImportError:
    pass


def params_from_list(params):
    return {"thickness": params[0], "angle_fact": params[1], "camber_angle": params[2]}


def generate_random_params():
    thickness = np.random.uniform(0.08, 0.12)
    lead_angle_fact = np.random.uniform(1, 10)
    camber_angle = np.random.uniform(0.1, 0.5)
    return {
        "thickness": thickness,
        "angle_fact": lead_angle_fact,
        "camber_angle": camber_angle,
    }


def generate_rotor(params):
    thickness = params["thickness"]
    lead_angle_fact = params["angle_fact"]
    camber_angle = params["camber_angle"]

    rotor_params = RotorParameters(
        tot_length=3,
        n_blades=4,
        blade_profile_type=FLAT,
        blade_camber_position=0.4,
        blade_thickness=thickness,
        blade_lead_angle_func=lambda r: atan(lead_angle_fact * r),
        blade_camber_angle_func=lambda r: camber_angle,
        blade_max_radius=1.5,
        blade_clearance=2.1 * thickness,
        hub_radius=0.4,
        hub_front_angle=0.5,
        hub_back_angle=0.2,
        shroud_thickness=0.05,
        shroud_slant_angle=1,
        shroud_in_fillet_radius=0.01,
        shroud_out_fillet_radius=0.02,
    )
    fact = 1.0
    while True:
        try:
            rotor_solid = (
                RotorBuilder(rotor_params)
                .add_hub()
                .add_shroud()
                .add_blades(151, 10)
                .get_occ_solid(fillet_radius=fact * thickness)
            )
            print("Fillet radius factor: {:.2g}".format(fact))
            break
        except FilletSolidError:
            fact *= 0.9
            if fact < 0.2:
                fact = 0.0
    return rotor_solid


def test_specific_parameters(
    params: list[list[float]] = [[0.08145885445708195, 4.35530262663131, 0.18463266097505393]]
):
    n_fails = 0
    success = []
    fails = []
    for i, p in enumerate(params):
        params = params_from_list(p)
        try:
            rotor_solid = generate_rotor(params)
            print("Success: ", i, params)
            success.append(list(params.values()))
        except Exception as e:
            n_fails += 1
            print(f"Failed(tot:{n_fails}): ", params, e)
            fails.append(list(params.values()))
    return success, fails, n_fails, rotor_solid


def test_random_parameters(k):
    n_fails = 0
    success = []
    fails = []
    rotor_solid = None
    for i in range(k):
        params = generate_random_params()
        try:
            rotor_solid = generate_rotor(params)
            print("Success: ", i, params)
            success.append(list(params.values()))
        except Exception as e:
            n_fails += 1
            print(f"Failed(tot:{n_fails}): ", params, e)
            fails.append(list(params.values()))
    return success, fails, n_fails, rotor_solid


if __name__ == "__main__":

    # success, fails, n_fails, rotor_solid = test_specific_parameters(
    #     [[0.05335743569040488, 4.088178148275992, 0.20591993210715318]]
    # )
    success, fails, n_fails, rotor_solid = test_random_parameters(500)

    print("Fails: ", fails)
    print("Failed: ", n_fails)

    if MATPLOTLIB:
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")

        for p1, p2, p3 in success:
            ax.scatter(p1, p2, p3, c="g")
        for p1, p2, p3 in fails:
            ax.scatter(p1, p2, p3, c="r")

        ax.set_xlabel("thickness")
        ax.set_ylabel("lead angle")
        ax.set_zlabel("camber angle")

        plt.show()

    save_shape_to_brep(rotor_solid, "rotor.brep")
    save_shape_to_step(rotor_solid, "rotor.step")

    display, start_display, _, _ = init_display()
    display.DisplayShape(rotor_solid, update=True)
    start_display()
