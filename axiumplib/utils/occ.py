from OCC.Core.TColgp import TColgp_Array2OfPnt, TColgp_Array1OfPnt, TColgp_HArray1OfPnt
from OCC.Core.BRepBuilderAPI import (
    BRepBuilderAPI_MakeEdge,
    BRepBuilderAPI_MakeWire,
    BRepBuilderAPI_MakeSolid,
    BRepBuilderAPI_Sewing,
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_Transform,
)
from OCC.Core.gp import gp_Trsf, gp_Ax1, gp_Pnt, gp_Dir, gp_Vec
from OCC.Core.TopoDS import (
    TopoDS_Shell,
    TopoDS_Face,
    TopoDS_Shape,
    TopoDS_Wire,
    TopoDS_Solid,
    TopoDS_Compound,
    TopoDS_Edge,
    TopoDS_Vertex,
)
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.BRepFilletAPI import BRepFilletAPI_MakeFillet2d
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE, TopAbs_WIRE, TopAbs_SOLID, TopAbs_SHAPE
from OCC.Core.TopTools import TopTools_ListOfShape
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.Geom import Geom_Curve
from axiumplib.glob_params import FUSE_TOL, FIXSOLID_PRECISION
from math import pi


def to_TColgp_Array1OfPnt(pts: list[list[float]]) -> TColgp_Array1OfPnt:
    """Convert a list of 3D points to a TColgp_Array1OfPnt."""
    points = [gp_Pnt(*pt) for pt in pts]
    return Tcol_dim_1(points, TColgp_Array1OfPnt)


def to_TColgp_HArray1OfPnt(pts: list[list[float]]) -> TColgp_HArray1OfPnt:
    """Convert a list of 3D points to a TColgp_HArray1OfPnt."""
    points = [gp_Pnt(*pt) for pt in pts]
    return Tcol_dim_1(points, TColgp_HArray1OfPnt)


def Tcol_dim_1(li, _type):
    pts = _type(1, len(li))
    for n, i in enumerate(li):
        pts.SetValue(n + 1, i)
    return pts


def to_TColgp_Array2OfPnt(uv_pts: list[list[list[float]]]) -> TColgp_Array2OfPnt:
    """Convert a 2D array of 3D points to a TColgp_Array2OfPnt."""
    points = [[gp_Pnt(*pt) for pt in u_pts] for u_pts in uv_pts]
    return Tcol_dim_2(points, TColgp_Array2OfPnt)


def Tcol_dim_2(li, _type):
    pts = _type(1, len(li), 1, len(li[0]))
    for i, ps in enumerate(li):
        for j, p in enumerate(ps):
            pts.SetValue(i + 1, j + 1, p)
    return pts


def wire_from_curves(curves: list[Geom_Curve]) -> TopoDS_Wire:
    """Create a wire from list of curves."""
    edges = []
    for curve in curves:
        edges.append(BRepBuilderAPI_MakeEdge(curve).Edge())
    return wire_from_edges(edges)


def wire_from_edges(edges: list[TopoDS_Edge]) -> TopoDS_Wire:
    """Create a wire from list of edges."""
    wire_builder = BRepBuilderAPI_MakeWire()
    for edge in edges:
        wire_builder.Add(edge)
    wire_builder.Build()
    wire_builder.Check()
    return wire_builder.Wire()


def solid_from_faces(faces: list[TopoDS_Face]) -> TopoDS_Solid:
    """Create a solid from list of faces."""
    # Sew the faces together to form a solid
    tol = 1e-6
    contin = True

    for _ in range(20):
        sewing = BRepBuilderAPI_Sewing(tol)
        for face in faces:
            sewing.Add(face)
        sewing.Perform()
        if isinstance(sewing.SewedShape(), TopoDS_Shell):
            contin = False
            if tol > 1e-3:
                print("Faces sewed with tol = {:.1e}".format(tol))
        else:
            tol *= 2
        if not contin:
            break

    if contin:
        raise ValueError("Failed to sew faces together.")

    solid_maker = BRepBuilderAPI_MakeSolid()
    solid_maker.Add(sewing.SewedShape())
    return solid_maker.Solid()


def solid_from_compound(compound: TopoDS_Compound) -> TopoDS_Solid:
    """Create a solid from a compound."""
    faces = get_faces_from_shape(compound)
    return solid_from_faces(faces)


def fix_solid(solid: TopoDS_Solid) -> TopoDS_Solid:
    from OCC.Core.ShapeFix import ShapeFix_Solid

    fixer = ShapeFix_Solid(solid)
    fixer.SetPrecision(FIXSOLID_PRECISION)
    fixer.Perform()
    return fixer.Solid()


def face_from_pts(pts: list[gp_Pnt] | TopTools_ListOfShape) -> TopoDS_Face:
    """Create a polygonal face from a list of coplanar points. No verification is done for coplanarity."""

    pts.append(pts[0])  # to make a loop
    edges = []
    for i in range(len(pts) - 1):
        edges.append(BRepBuilderAPI_MakeEdge(pts[i], pts[i + 1]).Edge())

    wire_maker = BRepBuilderAPI_MakeWire()
    for edge in edges:
        wire_maker.Add(edge)

    wire = wire_maker.Wire()
    return BRepBuilderAPI_MakeFace(wire, True).Face()


def get_vertices_from_shape(shape: TopoDS_Shape) -> list[TopoDS_Vertex]:
    """Extracts the vertices of a shape and returns them."""
    return get_elem_from_shape(shape, TopAbs_VERTEX)


def get_edges_from_shape(shape: TopoDS_Shape) -> list[TopoDS_Edge]:
    """Extracts the edges of a shape and returns them."""
    return get_elem_from_shape(shape, TopAbs_EDGE)


def get_faces_from_shape(shape: TopoDS_Shape) -> list[TopoDS_Face]:
    """Extracts the faces of a shape and returns them."""
    return get_elem_from_shape(shape, TopAbs_FACE)


def get_wires_from_shape(shape: TopoDS_Shape) -> list[TopoDS_Wire]:
    """Extracts the wires of a shape and returns them."""
    return get_elem_from_shape(shape, TopAbs_WIRE)


def get_solids_from_shape(shape: TopoDS_Shape) -> list[TopoDS_Solid]:
    """Extract the solids of a shape and returns them."""
    return get_elem_from_shape(shape, TopAbs_SOLID)


def get_elem_from_shape(shape: TopoDS_Shape, elemAbs) -> list[TopoDS_Shape]:
    elems = []
    explorer = TopExp_Explorer(shape, elemAbs)
    while explorer.More():
        elem = explorer.Current()
        elems.append(elem)
        explorer.Next()
    return elems


def fillet_face_vertices(face: TopoDS_Face, radii: float | list[float]):
    """Fillet the vertices of a face with a given radius, or one radius at each vertex."""
    vertices = get_vertices_from_shape(face)
    vertices = [vertices[i] for i in range(len(vertices)) if i % 2 == 0]  # removing duplicates
    if isinstance(radii, (float, int)):
        radii = [radii] * len(vertices)
    assert len(vertices) == len(radii), "Number of vertices and radii must match."
    fillet_maker = BRepFilletAPI_MakeFillet2d(face)
    for i, vertex in enumerate(vertices):
        if radii[i] > 0:
            fillet_maker.AddFillet(vertex, radii[i])
    return fillet_maker.Shape()


def translate(object: TopoDS_Shape, vector: list[float], copy: bool = False):
    """Translate an object by a vector."""
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(*vector))
    return BRepBuilderAPI_Transform(object, trsf, copy).Shape()


def rotation_array(
    object: TopoDS_Shape, n: int, angle: float = 2 * pi, point: list[float] = [0, 0, 0], axis: list[float] = [1, 0, 0]
):
    """Rotate an object n times around an axis."""
    rotation_axis = gp_Ax1(gp_Pnt(*point), gp_Dir(*axis))
    array = [object]
    for i in range(1, n):
        trsf = gp_Trsf()
        trsf.SetRotation(rotation_axis, i * angle / n)
        array.append(BRepBuilderAPI_Transform(object, trsf, True).Shape())
    return array


def boolean_union(shapes: list[TopoDS_Shape]) -> TopoDS_Shape:
    """Perform a Boolean union on a list of solids."""
    if not shapes:
        raise ValueError("The list of solids is empty.")
    objectShapes = TopTools_ListOfShape()
    objectShapes.Append(shapes[0])
    toolShapes = TopTools_ListOfShape()
    for shape in shapes[1:]:
        toolShapes.Append(shape)
    bool_fuse = BRepAlgoAPI_Fuse()
    bool_fuse.SetFuzzyValue(FUSE_TOL)
    bool_fuse.SetArguments(objectShapes)
    bool_fuse.SetTools(toolShapes)
    bool_fuse.SetRunParallel(True)
    bool_fuse.Build()
    try:
        bool_fuse.SimplifyResult()
    except:
        pass
    bool_fuse.Check()
    return bool_fuse.Shape()


def save_shape_to_brep(shape: TopoDS_Shape, filename: str):
    """Save the given shape to a BREP file."""
    from OCC.Core.BRepTools import breptools

    if not filename.endswith(".brep"):
        filename += ".brep"
    breptools.Write(shape, filename)


def save_shape_to_step(shape: TopoDS_Shape, filename: str):
    """Save the given shape to a STEP file."""
    from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs

    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)
    writer.Write(filename)
