from axiumplib.profile import ProfileBuilder, ProfileParameters
from axiumplib.utils.decorators import memoize
from axiumplib.utils.occ import to_TColgp_Array2OfPnt, to_TColgp_Array1OfPnt, wire_from_curves, solid_from_faces
from math import sin, atan, tan, cos
from scipy.optimize import fsolve
from json import load as jsonload
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.Core.GeomProjLib import geomprojlib
from OCC.Core.GeomAPI import GeomAPI_PointsToBSplineSurface, GeomAPI_PointsToBSpline, GeomAPI_Interpolate
from OCC.Core.Geom import Geom_CylindricalSurface, Geom_Transformation, Geom_Geometry
from OCC.Core.GeomAbs import GeomAbs_C2
from OCC.Core.ShapeFix import ShapeFix_Face
from OCC.Core.gp import gp_Ax3, gp_Ax1, gp_Pnt, gp_Dir
from axiumplib.glob_params import BLADE_APPROX_TOL
import numpy as np


class BladeBuilder:
    """A class to build a blade from a set of parameters.
    Returns a blade composed of two surface functions.
    """

    def __init__(self, params):
        self.params = params
        self.blade = None

    def create_blade(self):

        r = self.params.min_radius + 0.5 * (self.params.max_radius - self.params.min_radius)
        profilekwargs = self.params.profile_kwargs(r)
        mov_vec = profilekwargs["mov_vec"]
        profileparams = ProfileParameters(profile_type=self.params.profile_type, **profilekwargs)
        profile_builder = ProfileBuilder(profileparams)
        profile_builder.create_profile().bend_profile().rotate_profile().move_profile(mov_vec).wrap_on_X_axis_cylinder()
        profile_funcs = profile_builder.get_profile_funcs()

        def func_intrados(u, v):
            r = self.params.min_radius + v * (self.params.max_radius - self.params.min_radius)
            profilekwargs = self.params.profile_kwargs(r)
            return profile_funcs[0](u, **profilekwargs)

        def func_extrados(u, v):
            r = self.params.min_radius + v * (self.params.max_radius - self.params.min_radius)
            profilekwargs = self.params.profile_kwargs(r)
            return profile_funcs[1](u, **profilekwargs)

        self.blade = [func_intrados, func_extrados]

        return self

    def get_blade_funcs(self):
        return self.blade

    def get_occ_solid(self, u_points: int, v_points: int, u_func=lambda x: x, v_func=lambda x: x):
        """Generate the OCC solid representing the blade with cylinder caps."""

        intr_face = self._create_blade_face(
            intrados=True, u_points=u_points, v_points=v_points, u_func=u_func, v_func=v_func
        )
        extr_face = self._create_blade_face(
            intrados=False, u_points=u_points, v_points=v_points, u_func=u_func, v_func=v_func
        )

        top_cap = self._create_cylinder_cap(top=True, n_points=u_points)
        bot_cap = self._create_cylinder_cap(top=False, n_points=u_points)

        return solid_from_faces([intr_face, extr_face, top_cap, bot_cap])

    def _create_blade_face(self, intrados: bool, u_points: int, v_points: int, u_func=lambda x: x, v_func=lambda x: x):
        """Create intrados or extrados blade face."""
        func_face = self.blade[0] if intrados else self.blade[1]
        u_vals = u_func(np.linspace(0, 1, u_points))
        v_vals = v_func(np.linspace(0, 1, v_points))
        points = to_TColgp_Array2OfPnt([[func_face(u, v) for u in u_vals] for v in v_vals])
        surface = GeomAPI_PointsToBSplineSurface(points, 3, 8, GeomAbs_C2, BLADE_APPROX_TOL).Surface()
        face = BRepBuilderAPI_MakeFace(surface, 1e-6).Face()
        return face

    def _create_cylinder_cap(self, top: bool, n_points: int, func=lambda x: x):
        """Create a circular cap at the top or bottom of the blade."""
        # Cylinder
        radius = self.params.max_radius if top else self.params.min_radius
        cylinder_axis = gp_Ax3(gp_Pnt(), gp_Dir())
        cylinder_geom = Geom_CylindricalSurface(cylinder_axis, radius)

        # Sample points along the intrados and extrados at v=1 or v=0
        v = 1 if top else 0
        u_vals = func(np.linspace(0, 1, n_points))
        intr_points = to_TColgp_Array1OfPnt([self.blade[0](u, v) for u in u_vals])
        extr_points = to_TColgp_Array1OfPnt([self.blade[1](u, v) for u in u_vals[::-1]])  # Reversed for closed loop

        # Create edges from sampled points to form a loop
        intr_curve = GeomAPI_PointsToBSpline(intr_points, 3, 8, GeomAbs_C2, BLADE_APPROX_TOL).Curve()
        extr_curve = GeomAPI_PointsToBSpline(extr_points, 3, 8, GeomAbs_C2, BLADE_APPROX_TOL).Curve()

        if False:
            # Prjection may be needed sometimes... seems to work without it
            rotation_geom = Geom_Transformation()
            rotation_geom.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir()), np.pi)
            cylinder_geom.Transform(rotation_geom.Trsf())
            proj = geomprojlib()
            intr_curve = proj.Project(intr_curve, cylinder_geom)
            extr_curve = proj.Project(extr_curve, cylinder_geom)

        cap_wire = wire_from_curves([intr_curve, extr_curve])
        face_mkr = BRepBuilderAPI_MakeFace(cylinder_geom, cap_wire)  # Trimmed face

        if not face_mkr.IsDone():
            raise AssertionError("Could not make blade " + "top" if top else "bottom" + " cap.")

        # Don't know why but it is necessary to fix the face
        fix_face = ShapeFix_Face(face_mkr.Face())
        fix_face.SetPrecision(1e-6)
        fix_face.Perform()
        return fix_face.Face()


class BladeParameters:
    def __init__(
        self,
        profile_type="naca",
        blade_length=1,
        thickness=0.05,
        camber_position=0.4,
        min_radius=0.5,
        max_radius=1.5,
        axis=[1, 0, 0],
        origin=[0, 0, 0],
        lead_angle_func=lambda r: atan(r),
        camber_angle_func=lambda r: 0.1,
    ):
        self.profile_type = profile_type
        self.blade_length = blade_length
        self.thickness = thickness
        self.camber_position = camber_position
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.axis = axis
        self.origin = origin
        self.lead_angle_func = lead_angle_func
        self.camber_angle_func = camber_angle_func

    @memoize
    def profile_kwargs(self, r):
        lea, ca = self.lead_angle_func(r), self.camber_angle_func(r)
        max_camber, profile_angle = find_m_alpha(lea, lea - ca, self.camber_position)
        chord_length = self.blade_length / cos(profile_angle)
        half_width = self.thickness / (2 * chord_length)
        mov_vec = [0, -0.5 * chord_length * sin(profile_angle)]  # Centering around X axis
        return {
            "chord_length": chord_length,
            "half_width": half_width,
            "max_camber": max_camber,
            "camber_position": self.camber_position,
            "profile_angle": profile_angle,
            "mov_vec": mov_vec,
            "radius": r,
        }

    @classmethod
    def read(cls, file):
        params = cls()
        with open(file, "r") as f:
            config = jsonload(f)
            for key, value in config.items():
                setattr(params, key, value)
        return params


@memoize
def find_m_alpha(lea, tea, p):
    """Find the max camber and chord angle from leading edge angle,
    trailing edge angle, and camber position."""

    if np.isclose(lea, tea, 1e-4):
        return 0, lea

    def equation(X, lea, tea, p):
        m, chord_angle = X
        r0 = 2 * m / p - tan(lea - chord_angle)
        r1 = 2 * m / (p - 1) - tan(tea - chord_angle)
        return [r0, r1]

    m, chord_angle = fsolve(equation, [0.1, (lea + tea) / 2], args=(lea, tea, p))
    return m, chord_angle


if __name__ == "__main__":
    blade_params = BladeParameters(profile_type="flat")
    x_func = lambda x: x**3 * (6 * x**2 - 15 * x + 10)  # More precise at leading and trailing edges
    blade_solid = BladeBuilder(blade_params).create_blade().get_occ_solid(200, 20, x_func, x_func)

    from OCC.Display.SimpleGui import init_display

    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(blade_solid, update=True)
    start_display()
