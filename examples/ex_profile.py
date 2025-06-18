from axiumplib import ProfileBuilder, ProfileParameters
from axiumplib.utils.occ import to_TColgp_Array1OfPnt, to_TColgp_HArray1OfPnt
import numpy as np
from math import sin
from OCC.Core.GeomAPI import GeomAPI_PointsToBSpline, GeomAPI_Interpolate
from OCC.Display.SimpleGui import init_display

INTERP = 0
APPROX = 1
display, start_display, add_menu, add_function_to_menu = init_display()


def display_profile(params):
    for profile_func in get_profile_functions(params):
        bspline_curve = create_bspline_from_profile(profile_func)
        display.DisplayShape(bspline_curve, update=True)


def get_profile_functions(params):
    profile_builder = ProfileBuilder(params)
    mov_vec = [0, -0.5 * params.chord_length * sin(params.profile_angle)]  # Centering around X axis
    profile_builder.create_profile().bend_profile().rotate_profile().move_profile(
        mov_vec
    ).wrap_on_cylinder()
    return profile_builder.get_profile_funcs()


def get_points(func, npts):
    return [func(t) for t in np.linspace(0, 1, npts)]


def create_bspline_from_profile(profile_function, method=INTERP, num_points=300):
    pts = get_points(profile_function, num_points)
    if method == INTERP:
        pnts = to_TColgp_HArray1OfPnt(pts)
        intp = GeomAPI_Interpolate(pnts, False, 1e-4)
        intp.Perform()
        return intp.Curve()
    elif method == APPROX:
        pnts = to_TColgp_Array1OfPnt(pts)
        return GeomAPI_PointsToBSpline(pnts).Curve()
    else:
        raise ValueError("Unknown method")


if __name__ == "__main__":
    for radius in [0.1, 0.2, 0.5, 0.8, 1.0]:
        params = ProfileParameters(chord_length=1.5, radius=radius)
        display_profile(params)

    start_display()
