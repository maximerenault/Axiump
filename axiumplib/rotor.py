from math import atan
from axiumplib.hub import HubBuilder, HubParameters
from axiumplib.blade import BladeBuilder, BladeParameters, is_edge_at_radius, DEFAULT_SEAMS
from axiumplib.shroud import ShroudBuilder, ShroudParameters
from axiumplib.utils.occ import (
    boolean_union,
    rotation_array,
    translate,
    solid_from_compound,
    fix_solid,
    get_solids_from_shape,
    get_edges_from_shape,
    fillet_solid_edges,
)
from axiumplib.glob_params import NACA, FLAT


class RotorBuilder:
    def __init__(self, params):
        self.params = params
        self.parts = []
        self.fillet_edges = []

    def add_hub(self):
        hub = HubBuilder(self.params.hub_params).create_hub().get_occ_solid()
        self.parts.append(hub)
        return self

    def add_shroud(self):
        shroud = ShroudBuilder(self.params.shroud_params).create_shroud().get_occ_solid()
        shroud = translate(shroud, (self.params.hub_front_length, 0, 0))
        self.parts.append(shroud)
        return self

    def add_blades(
        self,
        u_points=100,
        v_points=30,
        u_func=lambda x: x**3 * (6 * x**2 - 15 * x + 10),
        v_func=lambda x: x**3 * (6 * x**2 - 15 * x + 10),
        u_seams=DEFAULT_SEAMS,
    ):
        n_blades = self.params.n_blades
        blade = BladeBuilder(self.params.blade_params).create_blade().get_occ_solid(u_points, v_points, u_func, v_func, u_seams)
        blade = translate(blade, (self.params.hub_front_length + self.params.blade_clearance / 2, 0, 0))
        blades = rotation_array(blade, n_blades)

        edges = [edge for i in range(n_blades) for edge in get_edges_from_shape(blades[i])]
        hub_edges = [edge for edge in edges if is_edge_at_radius(edge, self.params.hub_radius)]
        shroud_edges = [edge for edge in edges if is_edge_at_radius(edge, self.params.blade_max_radius)]
        self.fillet_edges.extend(hub_edges + shroud_edges)

        self.parts.extend(blades)
        return self

    def get_occ_solid(self, fillet_radius=0.0, get_fillet_edges=False):
        union, modified = boolean_union(self.parts, check_modified=self.fillet_edges)
        modified = [edge for edges in modified for edge in edges]
        if fillet_radius > 0:
            union = fillet_solid_edges(union, modified, fillet_radius)
        solids = get_solids_from_shape(union)
        if len(solids) == 1:
            solid = fix_solid(solids[0])
        else:
            solid = fix_solid(solid_from_compound(union))
        if get_fillet_edges:
            return solid, modified
        return solid


class RotorParameters:
    def __init__(
        self,
        tot_length=3,
        n_blades=4,
        blade_profile_type=FLAT,
        blade_camber_position=0.4,
        blade_thickness=0.05,
        blade_lead_angle_func=lambda r: atan(3 * r),
        blade_camber_angle_func=lambda r: 0.2,
        blade_max_radius=1.5,
        blade_clearance=0.12,  # gap around the blade, should be around 2*blade_thickness
        hub_type="biconic",
        hub_radius=0.4,
        hub_front_length=0.6,
        hub_back_length=1.2,
        hub_front_angle=0.5,
        hub_back_angle=0.2,
        hub_front_fillet_radius=0.04,
        hub_back_fillet_radius=0.08,
        shroud_thickness=0.05,
        shroud_slant_angle=1,
        shroud_in_fillet_radius=0.01,
        shroud_out_fillet_radius=0.02,
    ):
        self.tot_length = tot_length
        self.n_blades = n_blades
        self.blade_profile_type = blade_profile_type
        self.blade_camber_position = blade_camber_position
        self.blade_thickness = blade_thickness
        self.blade_lead_angle_func = blade_lead_angle_func
        self.blade_camber_angle_func = blade_camber_angle_func
        self.blade_max_radius = blade_max_radius
        self.blade_clearance = blade_clearance
        self.hub_type = hub_type
        self.hub_radius = hub_radius
        self.hub_front_length = hub_front_length
        self.hub_back_length = hub_back_length
        self.hub_front_angle = hub_front_angle
        self.hub_back_angle = hub_back_angle
        self.hub_front_fillet_radius = hub_front_fillet_radius
        self.hub_back_fillet_radius = hub_back_fillet_radius
        self.shroud_thickness = shroud_thickness
        self.shroud_slant_angle = shroud_slant_angle
        self.shroud_in_fillet_radius = shroud_in_fillet_radius
        self.shroud_out_fillet_radius = shroud_out_fillet_radius

    @property
    def blade_params(self):
        return BladeParameters(
            profile_type=self.blade_profile_type,
            blade_length=self.tot_length - self.hub_front_length - self.hub_back_length - self.blade_clearance,
            thickness=self.blade_thickness,
            camber_position=self.blade_camber_position,
            min_radius=self.hub_radius,
            max_radius=self.blade_max_radius,
            lead_angle_func=self.blade_lead_angle_func,
            camber_angle_func=self.blade_camber_angle_func,
        )

    @property
    def hub_params(self):
        return HubParameters(
            type=self.hub_type,
            tot_length=self.tot_length,
            radius=self.hub_radius,
            front_length=self.hub_front_length - 0.036,
            back_length=self.hub_back_length - 0.036,
            front_angle=self.hub_front_angle,
            back_angle=self.hub_back_angle,
            front_fillet_radius=self.hub_front_fillet_radius,
            back_fillet_radius=self.hub_back_fillet_radius,
        )

    @property
    def shroud_params(self):
        return ShroudParameters(
            in_length=self.tot_length - self.hub_front_length - self.hub_back_length,
            in_radius=self.blade_max_radius,
            thickness=self.shroud_thickness,
            slant_angle=self.shroud_slant_angle,
            in_fillet_radius=self.shroud_in_fillet_radius,
            out_fillet_radius=self.shroud_out_fillet_radius,
        )


if __name__ == "__main__":
    rotor_params = RotorParameters(blade_profile_type=FLAT)
    rotor_solid = RotorBuilder(rotor_params).add_hub().add_shroud().add_blades(61, 10).get_occ_solid(fillet_radius=0.05)
    from axiumplib.utils.occ import save_shape_to_brep

    save_shape_to_brep(rotor_solid, "rotor.brep")

    from OCC.Display.SimpleGui import init_display

    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(rotor_solid, update=True)
    start_display()
