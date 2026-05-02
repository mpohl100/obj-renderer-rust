use crate::camera::Camera;
use crate::camera::Ray;
use crate::coordinate_system::CoordinateSystem3D;
use crate::coordinate_system::SphericalCoordinates;
use crate::linear_optimizer::Objective;
use crate::scene::Object3D;
use crate::scene::ObjectLike;
use crate::scene::HasVertices;
use rs_math3d::FloatVector;
use rs_math3d::Vector3;
use rs_math3d::{Vec3d, Vector};
use rs_math3d::CrossProduct;

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
        CameraRay { ray, camera, width }
    }

    pub fn position_camera(&mut self, x: f64) {
        let factor = x * self.width;
        let new_target_point = self.ray.origin + self.ray.direction * factor;
        self.camera.position = new_target_point;
    }

    pub fn focal_point(&self) -> Vec3d {
        self.camera.focal_point()
    }

    pub fn camera(&self) -> &Camera {
        &self.camera
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

    pub fn calculate_intersection_direction_with_area(&self, other: Area) -> Vec3d {
        let direction = Vector3::<f64>::cross(&self.normal, &other.normal);
        if direction.length() < 1e-6 {
            return Vec3d::new(0.0, 0.0, 0.0); // Areas are parallel
        }
        direction.normalize()
    }

    pub fn rotate_vector(&self, vector: Vec3d, angle_degrees: f64) -> Vec3d {
        let angle_radians = angle_degrees.to_radians();
        let cos_angle = angle_radians.cos();
        let sin_angle = angle_radians.sin();
        let axis = self.normal.normalize();

        let rotated_vector = vector * cos_angle
            + Vector3::<f64>::cross(&axis, &vector) * sin_angle
            + axis * Vector3::<f64>::dot(&axis, &vector) * (1.0 - cos_angle);
        rotated_vector
    }
}

#[derive(Clone)]
pub struct CameraPositionOptimizer {
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
        );
        let camera_ray = CameraRay::new(camera);
        CameraPositionOptimizer {
            object3d,
            camera_ray,
        }
    }

    pub fn camera_ray(&self) -> &CameraRay {
        &self.camera_ray
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
        let _d = -Vector3::<f64>::dot(&normal, &center_of_mass);
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
            origin: *coordinate_system.origin(),
            direction: (tr_intersection.unwrap() - bl_intersection.unwrap()).normalize(),
        };

        // convert all bbox intersection points to the coordinate system
        let mut projected_points = Vec::new();
        for intersection in bbox_intersections {
            let local_point = coordinate_system.convert_point(&intersection);
            projected_points.push(local_point);
        }

        // calculate the projections if the projected points onto the diagonal ray
        let points_on_diagonal: Vec<f64> = projected_points
            .iter()
            .map(|p| Vector3::<f64>::dot(p, &diagonal_ray.direction))
            .collect();

        // calculate the t values of the points on the diagonal
        let t_values: Vec<f64> = points_on_diagonal
            .iter()
            .map(|p| {
                let origin_dot = Vector3::<f64>::dot(&diagonal_ray.origin, &diagonal_ray.direction);
                (*p - origin_dot) / diagonal_ray.direction.length()
            })
            .collect();

        // sort the t values
        let mut sorted_t_values = t_values.clone();
        sorted_t_values.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // calculate the t value of the tr_intersection
        let tr_local = coordinate_system.convert_point(&tr_intersection.unwrap());
        let tr_point_on_diagonal = Vector3::<f64>::dot(&tr_local, &diagonal_ray.direction);
        let origin_dot = Vector3::<f64>::dot(&diagonal_ray.origin, &diagonal_ray.direction);
        let tr_t_value = (tr_point_on_diagonal - origin_dot) / diagonal_ray.direction.length();

        // return the lowest t_value divided by the tr_t_value
        let distance = sorted_t_values[0] / tr_t_value;
        if distance.is_nan() {
            return f64::MAX;
        }
        if distance.is_infinite() {
            return f64::MAX;
        }
        distance
    }
}

#[derive(Clone)]
pub struct BlankSpaceRatios{
    top: f64,
    bottom: f64,
}

impl BlankSpaceRatios {
    pub fn new(top: f64, bottom: f64) -> Self {
        BlankSpaceRatios { top, bottom }
    }
}

#[derive(Clone)]
pub struct NewCameraPositionOptimizer<Shape: HasVertices + Clone + 'static> {
    marker: std::marker::PhantomData<Shape>,
    object: Box<dyn ObjectLike<Shape>>,
    camera: Camera,
}

impl<Shape: HasVertices + Clone + 'static> NewCameraPositionOptimizer<Shape> {
    pub fn new(camera: Camera, object_like: impl ObjectLike<Shape> + 'static, spherical_coordinates: SphericalCoordinates) -> Self {
        let object = Box::new(object_like);
        let center = object.center();
        let adjusted_spherical_coordinates = SphericalCoordinates {
            radius: object.bounding_box_radius(),
            theta: spherical_coordinates.theta,
            phi: spherical_coordinates.phi,
        };
        let target_point = center + adjusted_spherical_coordinates.to_cartesian();
        let mut positioned_camera = camera.clone();
        positioned_camera.locate(target_point, center);
        NewCameraPositionOptimizer {
            marker: std::marker::PhantomData,
            object,
            camera: positioned_camera,
        }
    }

    pub fn position_camera(&mut self, space_ratios: BlankSpaceRatios) -> Camera {
        let perpendicular_to_camera_area = Area::new(
            (self.camera.focal_point() - self.object.center()).normalize(),
            self.object.center(),
        );

        let perpendicular_to_up_direction_area = Area::new(
            self.camera.up.normalize(),
            self.object.center(),
        );

        let left_to_right_direction = perpendicular_to_camera_area.calculate_intersection_direction_with_area(perpendicular_to_up_direction_area);
        let top_to_bottom_direction = perpendicular_to_camera_area.rotate_vector(left_to_right_direction, 90.0);

        let ray_through_center = Ray {
            origin: self.object.center(),
            direction: top_to_bottom_direction.normalize(),
        };

        // calculate intersection points of the ray through center and the bounding sphere of the object
        let bounding_sphere_radius = self.object.bounding_sphere().radius();
        // we multiply the space ratio by two because the bounding sphere radius is only half the length of the object
        let up_point_of_fov = ray_through_center.origin + ray_through_center.direction * (bounding_sphere_radius + space_ratios.top * 2.0);
        let down_point_of_fov = ray_through_center.origin - ray_through_center.direction * (bounding_sphere_radius + space_ratios.bottom * 2.0);
        
        let new_look_at = (up_point_of_fov + down_point_of_fov) * 0.5;
        let new_look_at_to_current_position_ray = Ray {
            origin: new_look_at,
            direction: (self.camera.position - new_look_at).normalize(),
        };
        // the distance of the look at to the camera position is determined by the distance of up_point_of_fov to new_look_at being the sine of the fov
        let distance_to_camera = (up_point_of_fov - new_look_at).length() / ((self.camera.fov.to_radians() as f64) / 2.0).sin();
        let new_position = new_look_at + new_look_at_to_current_position_ray.direction * distance_to_camera;

        let mut new_camera = self.camera.clone();

        new_camera.locate(new_position, new_look_at);
        new_camera
    }
}   

