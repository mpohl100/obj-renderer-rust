use crate::linear_optimizer::Objective;
use crate::scene::Object3D;
use crate::coordinate_system::SphericalCoordinates;
use rs_math3d::{Vec3d, Vector, CrossProduct};
use rs_math3d::FloatVector;
use crate::camera::{self, Ray};
use crate::camera::Camera;

#[derive(Clone)]
pub struct CameraRay {
    ray: Ray,
    camera: Camera,
    width: f64,
}

impl CameraRay {
    pub fn new(camera: Camera) -> Self {
        let ray = camera.generate_ray(0.5, 0.5);
        let width = camera.distance_between_focal_point_and_image_plane();
        CameraRay {
            ray,
            camera,
            width,
        }
    }

    pub fn position_camera(&mut self, x: f64){
        let factor = x * self.width;
        let new_target_point = self.ray.origin + self.ray.direction * factor;
        self.camera.position = new_target_point;
    }
}   

#[derive(Clone)]
pub struct CameraPositionOptimizer{
    object3d: Object3D,
    camera_ray: CameraRay,
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
        let camera = Camera::new(
            target_point,
            center,
            Vec3d::new(0.0, 1.0, 0.0),
            45.0,
            1.0,
            0.1,
            1000.0,
        );
        let camera_ray = CameraRay::new(camera);
        CameraPositionOptimizer {
            object3d,
            camera_ray,
        }
    }

}

impl Objective for CameraPositionOptimizer {
    fn adjust(self, x: f64) -> Self {
        let mut new_camera_position_optimizer = self.clone();
        new_camera_position_optimizer.camera_ray.position_camera(x);
        new_camera_position_optimizer
    }

    fn evaluate(&self) -> f64 {
        0.0
    }
}