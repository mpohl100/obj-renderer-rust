use math3d::{Point3D, Vec3D};

/// @brief Represents a colored triangle in 3D space
/// @param vertices The three vertices of the triangle
/// @param color The color of the triangle
pub struct ColoredTriangle {
    pub vertices: [Point3D; 3],
    pub color: [f32; 3], // RGB
}

/// @brief Represents the scene containing triangles
/// @param triangles The triangles in the scene
/// @param coordinate_system The coordinate system (standard cartesian)
pub struct Scene {
    triangles: Vec<ColoredTriangle>,
    coordinate_system: CoordinateSystem3D,
}

use math3d::CoordinateSystem3D;

impl Scene {
    /// @brief Creates a new scene with the standard cartesian coordinate system
    /// @return Scene
    pub fn new() -> Self {
        Scene {
            triangles: Vec::new(),
            coordinate_system: CoordinateSystem3D::standard(),
        }
    }

    /// @brief Adds a colored triangle to the scene
    /// @param triangle The ColoredTriangle to add
    pub fn add_colored_triangle(&mut self, triangle: ColoredTriangle) {
        self.triangles.push(triangle);
    }

    /// @brief Returns a reference to the triangles in the scene
    /// @return Reference to Vec<ColoredTriangle>
    pub fn triangles(&self) -> &Vec<ColoredTriangle> {
        &self.triangles
    }

    /// @brief Returns a reference to the coordinate system
    /// @return Reference to CoordinateSystem3D
    pub fn coordinate_system(&self) -> &CoordinateSystem3D {
        &self.coordinate_system
    }
}
