use rs_math3d::CrossProduct;
use rs_math3d::Vector3;
use rs_math3d::{FloatVector, Vec3d, Vector};

/// @brief A right-handed 3D Cartesian coordinate system
///
/// Stores an origin and three basis axes. Basis axes are not required to
/// be orthonormal on construction; methods will normalize as needed.
#[derive(Clone)]
pub struct CoordinateSystem3D {
    origin: Vec3d,
    x_axis: Vec3d,
    y_axis: Vec3d,
    z_axis: Vec3d,
}

impl CoordinateSystem3D {
    /// @brief Create a new coordinate system from origin and basis axes
    /// @param origin world-space origin of the coordinate system
    /// @param x_axis basis vector pointing along the local X axis
    /// @param y_axis basis vector pointing along the local Y axis
    /// @param z_axis basis vector pointing along the local Z axis
    /// @return CoordinateSystem3D
    pub fn new(origin: Vec3d, x_axis: Vec3d, y_axis: Vec3d, z_axis: Vec3d) -> Self {
        CoordinateSystem3D {
            origin,
            x_axis,
            y_axis,
            z_axis,
        }
    }

    pub fn new_from_axes(origin: Vec3d, x_axis: Vec3d, y_axis: Vec3d) -> Self {
        let z_axis = Vector3::<f64>::cross(&x_axis, &y_axis).normalize();
        let y_axis_corrected = Vector3::<f64>::cross(&z_axis, &x_axis).normalize();
        CoordinateSystem3D {
            origin,
            x_axis: x_axis.normalize(),
            y_axis: y_axis_corrected,
            z_axis,
        }
    }

    /// @brief Create the standard Cartesian coordinate system at world origin
    pub fn standard() -> Self {
        CoordinateSystem3D::new(
            Vec3d::new(0.0, 0.0, 0.0),
            Vec3d::new(1.0, 0.0, 0.0),
            Vec3d::new(0.0, 1.0, 0.0),
            Vec3d::new(0.0, 0.0, 1.0),
        )
    }

    /// @brief Convert a world-space point into this coordinate system's local coordinates
    /// @param world_point A point expressed in world coordinates
    /// @return The point expressed in local coordinates (x, y, z)
    pub fn convert_point(&self, world_point: &Vec3d) -> Vec3d {
        // Translate into the coordinate system origin
        let rel = *world_point - self.origin;
        // Use normalized basis vectors to project
        let nx = self.x_axis.normalize();
        let ny = self.y_axis.normalize();
        let nz = self.z_axis.normalize();
        let x = Vec3d::dot(&rel, &nx);
        let y = Vec3d::dot(&rel, &ny);
        let z = Vec3d::dot(&rel, &nz);
        Vec3d::new(x, y, z)
    }

    /// @brief Get the world-space origin of this coordinate system
    pub fn origin(&self) -> &Vec3d {
        &self.origin
    }
}

pub struct SphericalCoordinates {
    pub radius: f64,
    pub theta: f64,
    pub phi: f64,
}

impl SphericalCoordinates {
    /// @brief Create new spherical coordinates
    pub fn new(radius: f64, theta: f64, phi: f64) -> Self {
        SphericalCoordinates { radius, theta, phi }
    }

    /// @brief Convert spherical coordinates to Cartesian coordinates
    pub fn to_cartesian(&self) -> Vec3d {
        let x = self.radius * self.theta.sin() * self.phi.cos();
        let y = self.radius * self.theta.sin() * self.phi.sin();
        let z = self.radius * self.theta.cos();
        Vec3d::new(x, y, z)
    }
}
