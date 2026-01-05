use crate::linear_optimizer::Objective;
use crate::scene::Object3D;
use crate::coordinate_system::SphericalCoordinates;
use rs_math3d::{Vec3d, Vector, CrossProduct};
use rs_math3d::Vector3;
use rs_math3d::FloatVector;
use crate::camera::{self, Ray};
use crate::camera::Camera;
use crate::coordinate_system::CoordinateSystem3D;

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

    pub fn focal_point(&self) -> Vec3d {
        self.camera.focal_point()
    }
}   

struct Area {
    normal: Vec3d,
    point: Vec3d,
}

impl Area {
    pub fn new(normal: Vec3d, point: Vec3d) -> Self {
        Area { normal, point }
    }

    pub fn calculate_intersection(&self, ray: &Ray) -> Option<Vec3d> {
        let denom = Vector3::<f64>::dot(&self.normal, &ray.direction);
        if denom.abs() < 1e-6 {
            return None; // Ray is parallel to the area
        }
        let d = Vector3::<f64>::dot(&self.normal, &self.point);
        let t = (d - Vector3::<f64>::dot(&self.normal, &ray.origin)) / denom;
        if t < 0.0 {
            return None; // Intersection is behind the ray origin
        }
        Some(ray.origin + ray.direction * t)
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
        // calculate the center of mass of the object
        let center_of_mass = self.object3d.center();
        // construct an area through the center of mass and whose normal is the vector from the center of mass and the camare focal point
        let camera_focal_point = self.camera_ray.camera.focal_point();
        let normal = (camera_focal_point - center_of_mass).normalize();
        let d = -Vector3::<f64>::dot(&normal, &center_of_mass);
        let area = Area::new(normal, center_of_mass);

        // calculate the intersection of the tl, tr, bl, br rays with the area
        let tl_ray = self.camera_ray.camera.generate_ray(0.0, 0.0);
        let tr_ray = self.camera_ray.camera.generate_ray(1.0, 0.0);
        let bl_ray = self.camera_ray.camera.generate_ray(0.0, 1.0);
        let br_ray = self.camera_ray.camera.generate_ray(1.0, 1.0);
        let tl_intersection = area.calculate_intersection(&tl_ray);
        let tr_intersection = area.calculate_intersection(&tr_ray);
        let bl_intersection = area.calculate_intersection(&bl_ray);
        let br_intersection = area.calculate_intersection(&br_ray);

        // calculate the intersections of the cube of the object3d with the area
        let (bbox_min_point, bbox_max_point) = self.object3d.bounding_box();
        let bbox_corners = vec![
            Vec3d::new(bbox_min_point.x, bbox_min_point.y, bbox_min_point.z),
            Vec3d::new(bbox_min_point.x, bbox_min_point.y, bbox_max_point.z),
            Vec3d::new(bbox_min_point.x, bbox_max_point.y, bbox_min_point.z),
            Vec3d::new(bbox_min_point.x, bbox_max_point.y, bbox_max_point.z),
            Vec3d::new(bbox_max_point.x, bbox_min_point.y, bbox_min_point.z),
            Vec3d::new(bbox_max_point.x, bbox_min_point.y, bbox_max_point.z),
            Vec3d::new(bbox_max_point.x, bbox_max_point.y, bbox_min_point.z),
            Vec3d::new(bbox_max_point.x, bbox_max_point.y, bbox_max_point.z),
        ];
        let mut bbox_intersections = Vec::new();
        for corner in bbox_corners {
            let ray = Ray {
                origin: corner,
                direction: normal,
            };
            if let Some(intersection) = area.calculate_intersection(&ray) {
                bbox_intersections.push(intersection);
            }
        }

        let coordinate_system = CoordinateSystem3D::new_from_axes(
            bl_intersection.unwrap(),
            br_intersection.unwrap() - bl_intersection.unwrap(),
            tl_intersection.unwrap() - bl_intersection.unwrap(),
        );

        // calculate the diagonal ray of the intersection points through the origin of the coordinate system
        let diagonal_ray = Ray {
            origin: coordinate_system.origin().clone(),
            direction: (tr_intersection.unwrap() - bl_intersection.unwrap()).normalize(),
        };

        0.0
    }
}