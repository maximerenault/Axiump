from math import sqrt, atan, sin, cos
from axiumplib.utils.geom import wrap_point_on_cylinder, wrap_point_on_X_axis_cylinder, rotate_2D_pt
from json import load as jsonload


class ProfileBuilder:
    def __init__(self, params):
        self.params = params
        self.profile = None

    def create_profile(self):
        # Generate the initial 2D profile
        w, c = self.params.profile_params
        if self.params.profile_type == "naca":
            profile_func = naca
        elif self.params.profile_type == "flat":
            profile_func = flat_ellipse
        self.profile = [
            lambda t, half_width=w, chord_length=c: profile_func(t, half_width, chord_length, top=True),
            lambda t, half_width=w, chord_length=c: profile_func(t, half_width, chord_length, top=False),
        ]
        return self

    def bend_profile(self):
        # Apply bending to the profile
        m, p, c = self.params.bend_params
        func0, func1 = self.profile
        self.profile = [
            lambda t, max_camber=m, camber_position=p, chord_length=c, **kwargs: naca_bend(
                func0(t, chord_length=chord_length, **kwargs), max_camber, camber_position, chord_length
            ),
            lambda t, max_camber=m, camber_position=p, chord_length=c, **kwargs: naca_bend(
                func1(t, chord_length=chord_length, **kwargs), max_camber, camber_position, chord_length
            ),
        ]
        return self

    def rotate_profile(self):
        # Rotate the profile based on the specified angle
        alpha = self.params.profile_angle
        func0, func1 = self.profile
        self.profile = [
            lambda t, profile_angle=alpha, **kwargs: rotate_2D_pt(func0(t, **kwargs), profile_angle),
            lambda t, profile_angle=alpha, **kwargs: rotate_2D_pt(func1(t, **kwargs), profile_angle),
        ]
        return self

    def move_profile(self, vec):
        # Move the profile by a vector
        func0, func1 = self.profile
        self.profile = [
            lambda t, mov_vec=vec, **kwargs: [cf + cv for (cf, cv) in zip(func0(t, **kwargs), mov_vec)],
            lambda t, mov_vec=vec, **kwargs: [cf + cv for (cf, cv) in zip(func1(t, **kwargs), mov_vec)],
        ]
        return self

    def wrap_on_cylinder(self):
        # Wrap the profile onto a cylinder. Converts 2D pts to 3D pts.
        r, a, o = self.params.cylinder_params
        func0, func1 = self.profile
        self.profile = [
            lambda t, radius=r, axis=a, origin=o, **kwargs: wrap_point_on_cylinder(
                func0(t, **kwargs), axis, radius, origin
            ),
            lambda t, radius=r, axis=a, origin=o, **kwargs: wrap_point_on_cylinder(
                func1(t, **kwargs), axis, radius, origin
            ),
        ]
        return self

    def wrap_on_X_axis_cylinder(self):
        # Wrap the profile onto a cylinder on the X axis. Converts 2D pts to 3D pts.
        r = self.params.radius
        func0, func1 = self.profile
        self.profile = [
            lambda t, radius=r, **kwargs: wrap_point_on_X_axis_cylinder(func0(t, **kwargs), radius),
            lambda t, radius=r, **kwargs: wrap_point_on_X_axis_cylinder(func1(t, **kwargs), radius),
        ]
        return self

    def get_profile_funcs(self):
        return self.profile


class ProfileParameters:
    def __init__(
        self,
        profile_type="naca",
        chord_length=1,
        half_width=0.025,
        max_camber=0.01,
        camber_position=0.4,
        profile_angle=1,
        radius=1,
        axis=[1, 0],
        origin=[0, 0],
        mov_vec=[0, 0],
    ):
        self.profile_type = profile_type
        self.chord_length = chord_length
        self.half_width = half_width
        self.max_camber = max_camber
        self.camber_position = camber_position
        self.profile_angle = profile_angle
        self.radius = radius
        self.axis = axis
        self.origin = origin
        self.mov_vec = mov_vec

    @property
    def profile_params(self):
        return self.half_width, self.chord_length

    @property
    def bend_params(self):
        return self.max_camber, self.camber_position, self.chord_length

    @property
    def cylinder_params(self):
        return self.radius, self.axis, self.origin

    @classmethod
    def read(cls, file):
        params = cls()
        with open(file, "r") as f:
            config = jsonload(f)
            for key, value in config.items():
                setattr(params, key, value)
        return params


def flat_ellipse(t, w=0.12, c=1.0, top=True):
    """Parametric equation for a flat ellipse profile.

    Args:
        t (float): Parameter value between 0 and 1.
        w (float): Half width of the profile, relative to the chord (half aspect ratio).
        c (float): Chord length of the profile.
        top (bool): True if the desired profile is the top one, False otherwise.

    Returns:
        np.array: 2D point on the profile.
    """
    assert c > 4 * w, "Cannot generate flat ellipse profile if c < 4*w !"
    assert t >= 0 and t <= 1, "t must be between 0 and 1 !"
    a = 2 * w * c
    b = w * c

    x = t * c
    if x < a:
        y = b * sqrt(1 - (x - a) ** 2 / a**2)
    elif x < c - a:
        y = w * c
    else:
        y = b * sqrt(1 - (x - c + a) ** 2 / a**2)

    if not top:
        y *= -1

    return x, y


def naca(t, w=0.12, c=1.0, top=True, close_trail=True):
    """Parametric equation for NACA 2-digit airfoil."""

    param4 = 0.1015  # Classic NACA
    if close_trail:
        param4 = 0.1036  # Closed trailing edge

    x = t * c
    y = 5 * w * c * (0.2969 * sqrt(t) - 0.1260 * t - 0.3516 * t**2 + 0.2843 * t**3 - param4 * t**4)

    if not top:
        y *= -1

    return x, y


def naca_bend(point, m=0.02, p=0.4, c=1.0):
    """Apply NACA 4-digit bending to a point."""
    x, y = point
    assert x >= 0 and x <= c, "x coordinate must be between 0 and c !"

    if x < p * c:
        yc = m * c * (x / c) * (2 * p - x / c) / (p**2)
        dyc_dx = 2 * m / p**2 * (p - x / c)
    else:
        yc = m * c * (c - x) / c * (1 + x / c - 2 * p) / ((1 - p) ** 2)
        dyc_dx = 2 * m / (1 - p) ** 2 * (p - x / c)

    theta = atan(dyc_dx)
    xb = x - y * sin(theta)
    yb = yc + y * cos(theta)

    return xb, yb
