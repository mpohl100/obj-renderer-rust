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
    pub triangles: Vec<ColoredTriangle>,
    pub coordinate_system: CoordinateSystem3D,
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
}
