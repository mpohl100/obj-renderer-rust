use crate::linear_optimizer::Objective;
use crate::scene::Object3D;
use crate::coordinate_system::SphericalCoordinates;
use rs_math3d::{Vec3d, Vector, CrossProduct};
use rs_math3d::FloatVector;
use crate::camera::Ray;

#[derive(Clone)]
pub struct CameraPositionOptimizer{
    object3d: Object3D,
    camera_ray: Ray,
}

impl CameraPositionOptimizer {

    pub fn new(object3d: Object3D, spherical_coordinates: SphericalCoordinates) -> Self {
        let center = object3d.center();
        let adjusted_spherical_coordinates = SphericalCoordinates {
            radius: object3d.bounding_box_radius(),
            theta: spherical_coordinates.theta,
            phi: spherical_coordinates.phi,
        };
        let target_point = center + adjusted_spherical_coordinates.to_cartesian();
        let center_ray = Ray {
            origin: target_point,
            direction: (center - target_point).normalize(),
        };
        CameraPositionOptimizer {
            object3d,
            camera_ray: center_ray,
        }
    }

}

impl Objective for CameraPositionOptimizer {
    fn adjust(self, x: f64) -> Self {
        self.clone()
    }

    fn evaluate(&self) -> f64 {
        0.0
    }
}