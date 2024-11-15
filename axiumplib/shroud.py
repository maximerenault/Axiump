from OCC.Core.gp import gp_Pnt, gp_Ax1, gp_Dir
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeRevol
from axiumplib.utils.occ import face_from_pts, fillet_face_vertices
from math import tan


class ShroudBuilder:
    def __init__(self, params):
        self.__params = params
        self.__shroud = None

    def create_shroud(self):
        """Create a shroud by revolving a trapezoidal profile around the X-axis."""
        in_length = self.__params.in_length
        in_radius = self.__params.in_radius
        thickness = self.__params.thickness
        slant_angle = self.__params.slant_angle
        in_fillet_radius = self.__params.in_fillet_radius
        out_fillet_radius = self.__params.out_fillet_radius
        out_radius = in_radius + thickness
        slant_length = thickness / tan(slant_angle)

        # Define the trapezoidal profile points
        pts = []
        pts.append(gp_Pnt(0, 0, in_radius))  # Inner radius, start
        pts.append(gp_Pnt(slant_length, 0, out_radius))  # Outer radius, start
        pts.append(gp_Pnt(in_length - slant_length, 0, out_radius))  # Outer radius, end
        pts.append(gp_Pnt(in_length, 0, in_radius))  # Inner radius, end

        profile_face = face_from_pts(pts)
        fillet_face = fillet_face_vertices(
            profile_face, [in_fillet_radius, out_fillet_radius, out_fillet_radius, in_fillet_radius]
        )
        axis = gp_Ax1(gp_Pnt(), gp_Dir())  # X-axis
        self.__shroud = BRepPrimAPI_MakeRevol(fillet_face, axis).Shape()

        return self

    def get_occ_solid(self):
        """Return the generated shroud solid."""
        return self.__shroud


class ShroudParameters:
    def __init__(
        self,
        in_length=1,
        in_radius=1,
        thickness=0.05,
        slant_angle=1.2,
        in_fillet_radius=0.01,
        out_fillet_radius=0.02,
    ):
        self.in_length = in_length
        self.in_radius = in_radius
        self.thickness = thickness
        self.slant_angle = slant_angle
        self.in_fillet_radius = in_fillet_radius
        self.out_fillet_radius = out_fillet_radius


if __name__ == "__main__":
    shroud_params = ShroudParameters()
    shroud_solid = ShroudBuilder(shroud_params).create_shroud().get_occ_solid()

    from OCC.Display.SimpleGui import init_display

    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(shroud_solid, update=True)
    start_display()
