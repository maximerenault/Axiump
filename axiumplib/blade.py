from axiumplib.profile import ProfileBuilder, ProfileParameters
from axiumplib.utils.decorators import memoize
from axiumplib.utils.occ import (
    to_TColgp_Array2OfPnt,
    to_TColgp_Array1OfPnt,
    to_TColgp_HArray1OfPnt,
    wire_from_curves,
    wire_from_edges,
    solid_from_faces,
    get_edges_from_shape,
    get_vertices_from_shape,
)
from math import sin, atan, tan, cos
from scipy.optimize import fsolve
from json import load as jsonload
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCC.Core.BRep import BRep_Tool
from OCC.Core.GeomProjLib import geomprojlib
from OCC.Core.GeomAPI import GeomAPI_PointsToBSplineSurface, GeomAPI_PointsToBSpline, GeomAPI_Interpolate
from OCC.Core.Geom import Geom_CylindricalSurface, Geom_Transformation
from OCC.Core.GeomAbs import GeomAbs_C2
from OCC.Core.ShapeFix import ShapeFix_Face
from OCC.Core.gp import gp_Ax3, gp_Ax1, gp_Pnt, gp_Dir
from OCC.Core.TopoDS import TopoDS_Face, TopoDS_Solid
from axiumplib.glob_params import (
    BLADE_APPROX_TOL,
    FIXFACE_PRECISION,
    INTERP_TOL,
    BLADE_CAP_RADIUS_TOL,
    MAKEFACE_TOL,
    FLAT,
    NACA,
)
from types import FunctionType
import numpy as np
from math import pi

INTERP = 0
APPROX = 1
DEFAULT_SEAMS = [0.25, 0.75]


class BladeBuilder:
    """A class to build a blade from a set of parameters.
    Returns a blade composed of two surface functions.
    """

    def __init__(self, params: "BladeParameters"):
        if not isinstance(params, BladeParameters):
            raise TypeError("params must be an instance of BladeParameters")
        self.__params = params
        self.__blade = None

    def create_blade(self) -> "BladeBuilder":
        """Create the blade surface functions."""
        min_radius = self.__params.min_radius
        max_radius = self.__params.max_radius
        profile_params_func = self.__params.profile_kwargs

        def get_radius(v):
            return min_radius + v * (max_radius - min_radius)

        def get_profile_kwargs(v):
            return profile_params_func(get_radius(v))

        profileparams = ProfileParameters(profile_type=self.__params.profile_type, **get_profile_kwargs(0.5))
        profile_builder = ProfileBuilder(profileparams)
        profile_builder.create_profile().bend_profile().rotate_profile().move_profile(
            get_profile_kwargs(0.5)["mov_vec"]
        ).wrap_on_X_axis_cylinder()

        profile_funcs = profile_builder.get_profile_funcs()

        self.__blade = [
            lambda u, v: profile_funcs[0](u, **get_profile_kwargs(v)),
            lambda u, v: profile_funcs[1](u, **get_profile_kwargs(v)),
        ]
        return self

    def get_blade_funcs(self) -> list[FunctionType]:
        return self.__blade

    def blade_func(self, u: float, v: float) -> list[float]:
        """Unifies intrados and extrados functions into one.
        u: [0, 1] normalized chordwise position, u<0.5 intrados, u>0.5 extrados, from head to tail and tail to head.
        v: [0, 1] normalized spanwise position."""
        return self.__blade[0](2 * u, v) if u < 0.5 else self.__blade[1](2 * (1 - u), v)

    def get_occ_solid(
        self,
        u_points: int,
        v_points: int,
        u_func: FunctionType = lambda x: x,
        v_func: FunctionType = lambda x: x,
        u_seams: list[float] = DEFAULT_SEAMS,
    ) -> TopoDS_Solid:
        """
        Generate the OCC solid representing the blade with cylinder caps.

        Parameters:
        - u_points: int, number of points along the chordwise direction.
        - v_points: int, number of points along the spanwise direction.
        - u_func: callable, distribution function for chordwise points.
        - v_func: callable, distribution function for spanwise points.
        - u_seams: list[float], seam positions along the chordwise direction.

        Returns:
        - TopoDS_Solid: the generated blade solid.
        """
        blade_faces = self._create_blade_faces(u_points, v_points, u_func, v_func, u_seams=u_seams)
        top_cap = self._create_cylinder_cap(blade_faces, self.__params.max_radius)
        bottom_cap = self._create_cylinder_cap(blade_faces, self.__params.min_radius)
        return solid_from_faces(blade_faces + [top_cap, bottom_cap])

    def _create_blade_faces(
        self,
        u_points: int,
        v_points: int,
        u_func: FunctionType = lambda x: x,
        v_func: FunctionType = lambda x: x,
        method: int = APPROX,
        u_seams: list[float] = DEFAULT_SEAMS,
    ) -> list[TopoDS_Face]:
        """
        Create blade face(s) based on seams in the chordwise direction.

        Parameters:
        - u_points: int, number of chordwise sampling points.
        - v_points: int, number of spanwise sampling points.
        - u_func: callable, distribution function for chordwise points.
        - v_func: callable, distribution function for spanwise points.
        - method: int, method for surface generation (INTERP or APPROX).
        - u_seams: list[float], seam positions in the chordwise direction.

        Returns:
        - list[TopoDS_Face]: generated blade faces.
        """
        if not u_seams:
            raise ValueError("At least one seam is required.")

        u_seams.sort()
        u_vals = np.concatenate(
            [u_func(np.linspace(0, 1, u_points)) / 2, u_func(np.linspace(0, 1, u_points))[1:-1] / 2 + 0.5]
        )

        for seam in u_seams:
            if seam not in u_vals:
                u_vals = np.sort(np.append(u_vals, seam))

        v_vals = v_func(np.linspace(0, 1, v_points))
        faces = []

        for i in range(len(u_seams)):
            if i == 0:
                loc_u_vals = np.concatenate([u_vals[u_vals >= u_seams[-1]], u_vals[u_vals <= u_seams[0]]])
            else:
                loc_u_vals = u_vals[(u_vals >= u_seams[i - 1]) & (u_vals <= u_seams[i])]

            # Generate points and create BSpline surfaces
            points = to_TColgp_Array2OfPnt([[self.blade_func(u, v) for u in loc_u_vals] for v in v_vals])
            pts_to_bsplinesurf = GeomAPI_PointsToBSplineSurface()
            if method == APPROX:
                pts_to_bsplinesurf.Init(points, 3, 8, GeomAbs_C2, BLADE_APPROX_TOL)
            elif method == INTERP:
                pts_to_bsplinesurf.Interpolate(points, False)
            else:
                raise ValueError(f"Unknown method: {method}")

            surface = pts_to_bsplinesurf.Surface()
            faces.append(BRepBuilderAPI_MakeFace(surface, MAKEFACE_TOL).Face())

        return faces

    def _create_cylinder_cap_resample(
        self,
        top: bool,
        n_points: int,
        func: FunctionType = lambda x: x,
        method: int = INTERP,
        seams: list[float] = DEFAULT_SEAMS,
    ) -> TopoDS_Face:
        """DEPRECATED. Create a circular cap at the top or bottom of the blade."""
        assert len(seams) > 0, "At least one seam is needed."
        seams.sort()

        # Cylinder
        radius = self.__params.max_radius if top else self.__params.min_radius
        cylinder_axis = gp_Ax3(gp_Pnt(), gp_Dir())
        cylinder_geom = Geom_CylindricalSurface(cylinder_axis, radius)

        # Sample points along the intrados and extrados at v=1 or v=0
        v = 1 if top else 0
        u_vals = func(np.linspace(0, 1, n_points)) / 2
        u_vals = np.concatenate([u_vals, u_vals[1:-1] + 0.5])
        for seam in seams:
            if seam not in u_vals:
                u_vals = np.sort(np.concatenate([u_vals, [seam]]))

        periodic = False
        curves = []
        for i, _ in enumerate(seams):
            if i == 0:
                loc_u_vals = np.concatenate([u_vals[u_vals >= seams[i - 1]], u_vals[u_vals <= seams[i]]])
            else:
                loc_u_vals = u_vals[np.logical_and(u_vals >= seams[i - 1], u_vals <= seams[i])]
            if method == INTERP:
                if len(seams) == 1:
                    loc_u_vals, periodic = loc_u_vals[:-1], True
                points = to_TColgp_HArray1OfPnt([self.blade_func(u, v) for u in loc_u_vals])
                interp = GeomAPI_Interpolate(points, periodic, INTERP_TOL)
                interp.Perform()
                curves.append(interp.Curve())
            elif method == APPROX:
                points = to_TColgp_Array1OfPnt([self.blade_func(u, v) for u in loc_u_vals])
                curves.append(GeomAPI_PointsToBSpline(points, 3, 8, GeomAbs_C2, BLADE_APPROX_TOL).Curve())
            else:
                raise ValueError("Unknown method")

        if False:
            # Prjection may be needed sometimes... seems to work without it
            rotation_geom = Geom_Transformation()
            rotation_geom.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir()), pi)
            cylinder_geom.Transform(rotation_geom.Trsf())
            proj = geomprojlib()
            for i in range(len(curves)):
                curves[i] = proj.Project(curves[i], cylinder_geom)

        cap_wire = wire_from_curves(curves)
        face_mkr = BRepBuilderAPI_MakeFace(cylinder_geom, cap_wire)  # Trimmed face

        if not face_mkr.IsDone():
            raise AssertionError("Could not make blade " + "top" if top else "bottom" + " cap.")

        # Don't know why but it is necessary to fix the face
        fix_face = ShapeFix_Face(face_mkr.Face())
        fix_face.SetPrecision(FIXFACE_PRECISION)
        fix_face.Perform()
        return fix_face.Face()

    def _create_cylinder_cap(self, blade_faces: list[TopoDS_Face], target_radius: float) -> TopoDS_Face:
        """
        Create a circular cap for the blade at the given radius.

        Parameters:
        - blade_faces: list[TopoDS_Face], faces of the blade.
        - target_radius: float, radius for the cap.

        Returns:
        - TopoDS_Face: generated cap face.
        """
        # Extract all edges from the blade faces
        edges = [edge for face in blade_faces for edge in get_edges_from_shape(face)]

        cap_edges = [edge for edge in edges if is_edge_at_radius(edge, target_radius)]
        if not cap_edges:
            raise ValueError(f"No edges found at radius {target_radius}.")

        # Create a wire and cap face
        cap_wire = wire_from_edges(cap_edges)
        cap_surface = Geom_CylindricalSurface(gp_Ax3(gp_Pnt(), gp_Dir()), target_radius)
        face_maker = BRepBuilderAPI_MakeFace(cap_surface, cap_wire)

        if not face_maker.IsDone():
            raise ValueError(f"Could not create cap at radius {target_radius}.")

        # Fix the face to ensure it is valid
        fixed_face = ShapeFix_Face(face_maker.Face())
        fixed_face.SetPrecision(FIXFACE_PRECISION)
        fixed_face.Perform()
        fixed_face.FixOrientation()
        return fixed_face.Face()


class BladeParameters:
    def __init__(
        self,
        profile_type: int = NACA,
        blade_length: float = 1,
        thickness: float = 0.05,
        camber_position: float = 0.4,
        min_radius: float = 0.5,
        max_radius: float = 1.5,
        axis: list[float] = [1, 0, 0],
        origin: list[float] = [0, 0, 0],
        lead_angle_func: FunctionType = lambda r: atan(r),
        camber_angle_func: FunctionType = lambda r: 0.1,
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
    def profile_kwargs(self, r: float) -> dict:
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
    def read(cls, file) -> "BladeParameters":
        params = cls()
        with open(file, "r") as f:
            config = jsonload(f)
            for key, value in config.items():
                setattr(params, key, value)
        return params

def is_edge_at_radius(edge, target_radius):
            for vertex in get_vertices_from_shape(edge):
                point = BRep_Tool.Pnt(vertex)
                radius = gp_Pnt(0, point.Y(), point.Z()).Distance(gp_Pnt())
                if abs(radius - target_radius) > BLADE_CAP_RADIUS_TOL:
                    return False
            return True

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
    blade_params = BladeParameters(profile_type=FLAT)
    x_func = lambda x: x**3 * (6 * x**2 - 15 * x + 10)  # More precise at leading and trailing edges
    blade_solid = BladeBuilder(blade_params).create_blade().get_occ_solid(41, 10, x_func, x_func)
    from OCC.Display.SimpleGui import init_display

    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(blade_solid, update=True)
    start_display()
