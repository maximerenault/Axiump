import os
import sys
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QVBoxLayout,
    QGroupBox,
    QHBoxLayout,
    QSlider,
    QLabel,
)
from PySide6.QtCore import Qt
from OCC.Display.backend import load_backend
from axiumplib import BladeParameters, BladeBuilder, FLAT, NACA

load_backend("pyside6")
import OCC.Display.qtDisplay as qtDisplay


class OCCParametricViewer(QDialog):
    def __init__(self):
        super().__init__()
        self.slider_precision = 0.01
        self.init_window()
        self.init_ui()

    def init_window(self):
        """Set up the main window properties."""
        self.setWindowTitle("OCC Parametric BSpline Surface Viewer")
        self.setGeometry(100, 100, 800, 600)

    def init_ui(self):
        """Initialize the UI components."""
        self.create_layout()

        # Set up layout for the dialog
        main_layout = QVBoxLayout()
        main_layout.addWidget(self.horizontalGroupBox)
        self.setLayout(main_layout)
        self.show()

        # Initialize the OCC display
        self.canvas.InitDriver()
        self.canvas.resize(200, 200)
        self.display = self.canvas._display
        self.update_surface()

    def create_layout(self):
        """Create the layout for OCC display, control buttons, and sliders."""
        self.horizontalGroupBox = QGroupBox("Parametric BSpline Surface Display")
        layout = QHBoxLayout()

        # Left-side controls for parameters
        control_layout = QVBoxLayout()

        # Set fixed width for the control panel
        control_container = QGroupBox()
        control_container.setLayout(control_layout)
        control_container.setFixedWidth(200)

        # Slider for first parameter
        self.param1_label = QLabel("Min radius")
        self.param1_slider = QSlider(Qt.Horizontal)
        self.param1_slider.setRange(0.05 / self.slider_precision, 0.7 / self.slider_precision)
        self.param1_slider.setValue(0.4 / self.slider_precision)
        self.param1_slider.valueChanged.connect(self.update_surface)
        control_layout.addWidget(self.param1_label)
        control_layout.addWidget(self.param1_slider)

        # Slider for second parameter
        self.param2_label = QLabel("Max radius")
        self.param2_slider = QSlider(Qt.Horizontal)
        self.param2_slider.setRange(0.8 / self.slider_precision, 2 / self.slider_precision)
        self.param2_slider.setValue(1 / self.slider_precision)
        self.param2_slider.valueChanged.connect(self.update_surface)
        control_layout.addWidget(self.param2_label)
        control_layout.addWidget(self.param2_slider)

        # Add control container with fixed width to the main layout
        layout.addWidget(control_container)

        # OCC 3D viewer
        self.canvas = qtDisplay.qtViewer3d(self)
        layout.addWidget(self.canvas, stretch=1)  # Allow canvas to expand
        self.horizontalGroupBox.setLayout(layout)

    def update_surface(self):
        blade_params = BladeParameters(
            profile_type=FLAT,
            min_radius=self.param1_slider.value() * self.slider_precision,
            max_radius=self.param2_slider.value() * self.slider_precision,
        )
        bb = BladeBuilder(blade_params)
        x_func = lambda x: x**3 * (6 * x**2 - 15 * x + 10)
        solid = bb.create_blade().get_occ_solid(71, 20, x_func, x_func)

        self.display.EraseAll()
        self.display.DisplayShape(solid, update=True)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = OCCParametricViewer()
    if os.getenv("APPVEYOR") is None:
        sys.exit(app.exec())
