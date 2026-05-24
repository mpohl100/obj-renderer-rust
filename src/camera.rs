use rs_math3d::CrossProduct;
use rs_math3d::FloatVector;
use rs_math3d::Vec3d;

/// @brief Represents a standard 3D camera
/// @param position The position of the camera in world coordinates
/// @param look_at The point the camera is looking at
/// @param up The up direction for the camera
/// @param fov Field of view in degrees
/// @param aspect_ratio Aspect ratio of the image
/// @param near Near clipping plane
/// @param far Far clipping plane
#[derive(Clone)]
pub struct Camera {
    pub position: Vec3d,
    pub look_at: Vec3d,
    pub up: Vec3d,
    pub fov: f32,
    pub aspect_ratio: f32,
}

impl Camera {
    /// @brief Creates a new camera oriented to look at the origin
    /// @param position Camera position
    /// @param look_at Look-at point
    /// @param up Up direction
    /// @param fov Field of view
    /// @param aspect_ratio Aspect ratio
    /// @return Camera
    pub fn new(position: Vec3d, look_at: Vec3d, up: Vec3d, fov: f32, aspect_ratio: f32) -> Self {
        Camera {
            position,
            look_at,
            up,
            fov,
            aspect_ratio,
        }
    }

    /// @brief Generates a ray from the camera through the pixel at normalized coordinates (u, v)
    /// @param u Horizontal normalized coordinate [0, 1]
    /// @param v Vertical normalized coordinate [0, 1]
    /// @return Ray struct with origin and direction
    pub fn generate_ray(&self, u: f32, v: f32) -> Ray {
        // Camera basis
        let forward = (self.look_at - self.position).normalize();
        let right = CrossProduct::cross(&forward, &self.up).normalize();
        let up = CrossProduct::cross(&right, &forward).normalize();
        // Image plane
        let fov_rad = self.fov.to_radians();
        let half_height = (fov_rad / 2.0).tan();
        let half_width = self.aspect_ratio * half_height;
        let px = ((2.0 * u - 1.0) * half_width) as f64;
        let py = ((1.0 - 2.0 * v) * half_height) as f64;
        let dir = (forward + right * px + up * py).normalize();
        Ray {
            origin: self.position,
            direction: dir,
        }
    }

    /// @brief Calculates the distance between the camera's focal point and the image plane
    pub fn distance_between_focal_point_and_image_plane(&self) -> f64 {
        1.0 / ((self.fov.to_radians() / 2.0).tan() as f64)
    }

    /// @brief Returns the focal point of the camera
    pub fn focal_point(&self) -> Vec3d {
        self.look_at
    }

    pub fn locate(&mut self, new_position: Vec3d, look_at: Vec3d) {
        self.position = new_position;
        self.look_at = look_at;
    }
}

/// @brief Represents a ray in 3D space
/// @param origin The origin of the ray
/// @param direction The direction of the ray (normalized)
#[derive(Clone)]
pub struct Ray {
    pub origin: Vec3d,
    pub direction: Vec3d,
}
