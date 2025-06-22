from OCC.Core.gp import gp_Pnt, gp_Ax1, gp_Dir
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeRevol
from axiumplib.utils.occ import (
    face_from_pts,
    fillet_face_vertices,
    boolean_union,
    fix_solid,
    solid_from_compound,
    rotation_array,
)
from math import tan, pi


class HubBuilder:
    def __init__(self, params):
        self.__params = params
        self.__hub = None

    def create_hub(self):
        """Create a hub by revolving a profile around the X-axis."""
        tot_length = self.__params.tot_length
        radius = self.__params.radius
        front_length = self.__params.front_length
        back_length = self.__params.back_length
        front_angle = self.__params.front_angle
        back_angle = self.__params.back_angle
        front_fillet_radius = self.__params.front_fillet_radius
        back_fillet_radius = self.__params.back_fillet_radius

        if self.__params.type == "biconic":
            height_front = radius - front_length * tan(front_angle)
            height_back = radius - back_length * tan(back_angle)

            assert height_front >= 0, "Front angle too large."
            assert height_back >= 0, "Back angle too large."

            # Define the profile points
            pts = []
            pts.append(gp_Pnt(0, 0, 0))
            pts.append(gp_Pnt(0, 0, height_front))
            pts.append(gp_Pnt(front_length, 0, radius))
            pts.append(gp_Pnt(tot_length - back_length, 0, radius))
            pts.append(gp_Pnt(tot_length, 0, height_back))
            pts.append(gp_Pnt(tot_length, 0, 0))

            # Mutliply by 0.9999 to avoid degenerate fillets
            radius_front = height_front / tan((pi / 2 - front_angle) / 2) * 0.9999
            radius_back = height_back / tan((pi / 2 - back_angle) / 2) * 0.9999

            profile_face = face_from_pts(pts)
            fillet_face = fillet_face_vertices(
                profile_face,
                [0, radius_front, front_fillet_radius, back_fillet_radius, radius_back, 0],
            )

            axis = gp_Ax1(gp_Pnt(), gp_Dir())  # X-axis
            third_hub = BRepPrimAPI_MakeRevol(fillet_face, axis, 2 * pi / 3).Shape()
            hub_parts = rotation_array(third_hub, 3)
            self.__hub = fix_solid(solid_from_compound(boolean_union(hub_parts)))

        else:
            raise ValueError(f"Unknown hub type: {self.__params.type}")

        return self

    def get_occ_solid(self):
        """Return the generated hub solid."""
        return self.__hub


class HubParameters:
    def __init__(
        self,
        type="biconic",
        tot_length=3,
        radius=0.4,
        front_length=0.6,
        back_length=1.2,
        front_angle=0.5,
        back_angle=0.2,
        front_fillet_radius=0.04,
        back_fillet_radius=0.08,
    ):
        self.type = type
        self.tot_length = tot_length
        self.radius = radius
        self.front_length = front_length
        self.back_length = back_length
        self.front_angle = front_angle
        self.back_angle = back_angle
        self.front_fillet_radius = front_fillet_radius
        self.back_fillet_radius = back_fillet_radius


if __name__ == "__main__":
    hub_params = HubParameters()
    hub_solid = HubBuilder(hub_params).create_hub().get_occ_solid()

    from OCC.Display.SimpleGui import init_display

    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(hub_solid, update=True)
    start_display()
