from OCC.Core.GeomAPI import GeomAPI_PointsToBSplineSurface
from OCC.Core.GeomAbs import GeomAbs_C2
from OCC.Display.SimpleGui import init_display
from axiumplib import BladeBuilder, BladeParameters, FLAT, NACA
import numpy as np
from axiumplib.utils.occ import to_TColgp_Array2OfPnt


def blade_to_bspl_surfaces(blade_params):
    blade_builder = BladeBuilder(blade_params).create_blade()
    top_surface, bottom_surface = blade_builder.get_blade_funcs()

    bspl_surface_top = create_bsplinesurface_from_blade(top_surface)
    bspl_surface_bot = create_bsplinesurface_from_blade(bottom_surface)
    return bspl_surface_top, bspl_surface_bot


def get_uv_points(uvfunc, us, vs):
    return [[uvfunc(u, v) for u in us] for v in vs]


def create_bsplinesurface_from_blade(surface_func, num_pts_u=101, num_pts_v=20):
    x_func = lambda x: x**3 * (6 * x**2 - 15 * x + 10)
    uv_points = get_uv_points(
        surface_func, x_func(np.linspace(0, 1, num_pts_u)), x_func(np.linspace(0, 1, num_pts_v))
    )
    array = to_TColgp_Array2OfPnt(uv_points)
    return GeomAPI_PointsToBSplineSurface(array, 3, 8, GeomAbs_C2, 0.001).Surface()


if __name__ == "__main__":
    blade_params = BladeParameters(profile_type=FLAT)
    surf_top, surf_bot = blade_to_bspl_surfaces(blade_params)

    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(surf_top, update=False)
    display.DisplayShape(surf_bot, update=True)
    start_display()
