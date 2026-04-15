use rs_math3d::FloatVector;
use rs_math3d::Vector;
use rs_math3d::{CrossProduct, Vec3d};

use std::sync::Arc;

use crate::camera::Camera;
use crate::camera::Ray;
use crate::coordinate_system::CoordinateSystem3D;

/// @brief Checks intersection of a ray with a triangle using Möller–Trumbore algorithm
/// @param ray The ray
/// @param vertices The triangle vertices
/// @return Some(distance) if hit, None otherwise
pub fn ray_triangle_intersect(
    ray: &crate::camera::Ray,
    vertices: &[rs_math3d::Vec3d; 3],
) -> Option<f64> {
    let v0 = vertices[0];
    let v1 = vertices[1];
    let v2 = vertices[2];
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = CrossProduct::cross(&ray.direction, &edge2);
    let a = Vec3d::dot(&edge1, &h);
    if a.abs() < 1e-6 {
        return None;
    }
    let f = 1.0 / a;
    let s = ray.origin - v0;
    let u = f * rs_math3d::Vec3d::dot(&s, &h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = rs_math3d::Vec3d::cross(&s, &edge1);
    let v = f * rs_math3d::Vec3d::dot(&ray.direction, &q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * rs_math3d::Vec3d::dot(&edge2, &q);
    if t > 1e-6 {
        Some(t)
    } else {
        None
    }
}

/// @brief Represents a colored triangle in 3D space
/// @param vertices The three vertices of the triangle
/// @param color The color of the triangle
#[derive(Clone)]
pub struct ColoredTriangle {
    pub vertices: [Vec3d; 3],
    pub color: [f32; 3], // RGB
}

impl ColoredTriangle {
    /// @brief Calculates the area of the triangle using the cross product method
    /// @return Area of the triangle
    pub fn area(&self) -> f64 {
        let edge1 = self.vertices[1] - self.vertices[0];
        let edge2 = self.vertices[2] - self.vertices[0];
        let cross_product = CrossProduct::cross(&edge1, &edge2);
        cross_product.length() * 0.5
    }
}

impl HasVertices for ColoredTriangle {
    fn vertices(&self) -> &[Vec3d] {
        &self.vertices
    }
}

#[derive(Clone)]
pub struct Sphere {
    pub center: Vec3d,
    pub radius: f64,
}

#[derive(Clone)]
pub struct ColoredSphere {
    pub sphere: Sphere,
    pub color: [f32; 3], // RGB
}

impl ColoredSphere {
    pub fn new(center: Vec3d, radius: f64, color: [f32; 3]) -> Self {
        ColoredSphere {
            sphere: Sphere { center, radius },
            color,
        }
    }
}

impl HasVertices for ColoredSphere {
    fn vertices(&self) -> &[Vec3d] {
        // A sphere does not have vertices in the same way a triangle does.
        // For the purpose of this trait, we can return an empty slice or
        // perhaps the center as a single "vertex".
        std::slice::from_ref(&self.sphere.center)
    }
}

pub trait HasVertices {
    fn vertices(&self) -> &[Vec3d];
}

struct ContainingSphere<Shape: HasVertices + Clone + 'static> {
    sphere: Sphere,
    contained_shapes: Vec<Shape>, // Indices of triangles contained within this sphere
}

impl<Shape: HasVertices + Clone + 'static> ContainingSphere<Shape> {
    fn new(center: Vec3d, radius: f64) -> Self {
        ContainingSphere {
            sphere: Sphere { center, radius },
            contained_shapes: Vec::new(),
        }
    }

    fn contains(&self, shape: &Shape) -> bool {
        // use this logic if Shape is of type ColoredTriangle
        if std::any::TypeId::of::<Shape>() == std::any::TypeId::of::<ColoredTriangle>() {
            let triangle = unsafe { &*(shape as *const Shape as *const ColoredTriangle) };
            for &vertex in &triangle.vertices {
                if (vertex - self.sphere.center).length() <= self.sphere.radius {
                    return true;
                }
            }
        } else if std::any::TypeId::of::<Shape>() == std::any::TypeId::of::<ColoredSphere>() {
            let colored_sphere = unsafe { &*(shape as *const Shape as *const ColoredSphere) };
            let center_distance = (colored_sphere.sphere.center - self.sphere.center).length();
            if center_distance + colored_sphere.sphere.radius <= self.sphere.radius {
                return true;
            }
        }
        false
    }

    fn add_shape(&mut self, shape: Shape) {
        if self.contains(&shape) {
            self.contained_shapes.push(shape);
        }
    }
}

#[derive(Clone)]
pub struct WrappedContainingSphere<Shape: HasVertices + Clone + 'static> {
    sphere: Arc<ContainingSphere<Shape>>,
}

impl<Shape: HasVertices + Clone + 'static> WrappedContainingSphere<Shape> {
    fn new(sphere: ContainingSphere<Shape>) -> Self {
        WrappedContainingSphere {
            sphere: Arc::new(sphere),
        }
    }

    fn _contains(&self, shape: &Shape) -> bool {
        self.sphere.contains(shape)
    }

    fn add_shape(&self, shape: Shape) {
        Arc::get_mut(&mut self.sphere.clone())
            .unwrap()
            .add_shape(shape);
    }
}

#[derive(Clone)]
pub struct Tile {
    pub start_x: u32,
    pub end_x: u32,
    pub start_y: u32,
    pub end_y: u32,
}

#[derive(Clone)]
struct Grid<Shape: HasVertices + Clone + 'static> {
    _phantom: std::marker::PhantomData<Shape>,
    min_point: Vec3d,
    cell_length: f64,
}

impl<Shape: HasVertices + Clone + 'static> Grid<Shape> {
    fn new(min_point: Vec3d, cell_length: f64) -> Self {
        Grid {
            _phantom: std::marker::PhantomData,
            min_point,
            cell_length,
        }
    }

    fn get_containing_sphere(&self, point: Vec3d) -> WrappedContainingSphere<Shape> {
        let x_index = ((point.x - self.min_point.x) / self.cell_length).floor() as i32;
        let y_index = ((point.y - self.min_point.y) / self.cell_length).floor() as i32;
        let z_index = ((point.z - self.min_point.z) / self.cell_length).floor() as i32;
        let center = Vec3d::new(
            self.min_point.x + (x_index as f64 + 0.5) * self.cell_length,
            self.min_point.y + (y_index as f64 + 0.5) * self.cell_length,
            self.min_point.z + (z_index as f64 + 0.5) * self.cell_length,
        );
        WrappedContainingSphere::new(ContainingSphere::new(
            center,
            self.cell_length * (3.0_f64).sqrt() / 2.0,
        ))
    }
}

pub trait ObjectLike<Shape: HasVertices + Clone + 'static> {
    fn radius(&self) -> f64;
    fn get_sphere(&self, point: Vec3d) -> WrappedContainingSphere<Shape>;
    fn deduce_hit_color(
        &self,
        ray: Ray,
        containing_sphere: &WrappedContainingSphere<Shape>,
    ) -> Option<[f32; 3]>;
    fn get_all_eight_corners_of_min_max_point(&self) -> [Vec3d; 8];
}

#[derive(Clone)]
pub struct Object3D {
    triangles: Vec<ColoredTriangle>,
    spheres: Vec<WrappedContainingSphere<ColoredTriangle>>,
    grid: Grid<ColoredTriangle>,
    _coordinate_system: CoordinateSystem3D,
    min_point: Vec3d,
    max_point: Vec3d,
    radius: f64,
}

impl Default for Object3D {
    fn default() -> Self {
        Self::new()
    }
}

impl Object3D {
    /// @brief Creates a new empty Object3D
    pub fn new() -> Self {
        Object3D {
            triangles: Vec::new(),
            spheres: Vec::new(),
            _coordinate_system: CoordinateSystem3D::standard(),
            min_point: Vec3d::new(0.0, 0.0, 0.0),
            max_point: Vec3d::new(0.0, 0.0, 0.0),
            grid: Grid::<ColoredTriangle>::new(Vec3d::new(0.0, 0.0, 0.0), 0.0),
            radius: 0.0,
        }
    }

    pub fn radius(&self) -> f64 {
        self.radius
    }

    pub fn triangles(&self) -> &Vec<ColoredTriangle> {
        &self.triangles
    }

    pub fn center(&self) -> Vec3d {
        (self.min_point + self.max_point) * 0.5
    }

    pub fn bounding_sphere_radius(&self) -> f64 {
        let center = self.center();
        let mut max_distance = 0.0;
        for tri in &self.triangles {
            for &v in &tri.vertices {
                let distance = (v - center).length();
                if distance > max_distance {
                    max_distance = distance;
                }
            }
        }
        max_distance
    }

    pub fn bounding_box_radius(&self) -> f64 {
        let center = self.center();
        (self.max_point - center).length()
    }

    pub fn bounding_box(&self) -> (Vec3d, Vec3d) {
        (self.min_point, self.max_point)
    }

    /// @brief Adds a colored triangle to the object 3D
    /// @param triangle The ColoredTriangle to add
    pub fn add_colored_triangle(&mut self, triangle: ColoredTriangle) {
        self.triangles.push(triangle);
    }

    /// @brief Positions spheres in the scene based on the bounding box of the triangles
    pub fn position_spheres(&mut self) {
        self.min_point = self.deduce_min_point();
        self.max_point = self.deduce_max_point();

        let avg_triangle_area = self.deduce_average_triangle_area();
        self.radius = avg_triangle_area.sqrt();
        let radius_times_sqrt_3 = self.radius * (3.0_f64).sqrt();
        self.grid = Grid::<ColoredTriangle>::new(self.min_point, radius_times_sqrt_3);

        let mut current_point = self.min_point;
        while current_point.x <= self.max_point.x {
            while current_point.y <= self.max_point.y {
                while current_point.z <= self.max_point.z {
                    // Here you would add a sphere at current_point with the calculated radius
                    current_point.z += self.radius * 2.0; // Move to the next position in z
                    self.spheres
                        .push(self.grid.get_containing_sphere(current_point));
                }
                current_point.y += self.radius * 2.0; // Move to the next position in y
                current_point.z = self.min_point.z; // Reset z to min
            }
            current_point.x += self.radius * 2.0; // Move to the next position in x
            current_point.y = self.min_point.y; // Reset y to min
            current_point.z = self.min_point.z; // Reset z to min
        }

        for tri in &self.triangles {
            for sphere in &mut self.spheres {
                if tri.vertices.iter().any(|&v| {
                    (v - sphere.sphere.sphere.center).length() <= sphere.sphere.sphere.radius
                }) {
                    sphere.add_shape(tri.clone());
                }
            }
        }
    }

    fn deduce_min_point(&self) -> Vec3d {
        let mut min_point = Vec3d::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        for tri in &self.triangles {
            for &v in &tri.vertices {
                min_point.x = min_point.x.min(v.x);
                min_point.y = min_point.y.min(v.y);
                min_point.z = min_point.z.min(v.z);
            }
        }
        min_point
    }

    fn deduce_max_point(&self) -> Vec3d {
        let mut max_point = Vec3d::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for tri in &self.triangles {
            for &v in &tri.vertices {
                max_point.x = max_point.x.max(v.x);
                max_point.y = max_point.y.max(v.y);
                max_point.z = max_point.z.max(v.z);
            }
        }
        max_point
    }

    fn deduce_average_triangle_area(&self) -> f64 {
        let mut total_area = 0.0;
        for tri in &self.triangles {
            total_area += tri.area();
        }
        total_area / self.triangles.len() as f64
    }
}

impl ObjectLike<ColoredTriangle> for Object3D {
    fn radius(&self) -> f64 {
        self.radius
    }

    fn get_sphere(&self, point: Vec3d) -> WrappedContainingSphere<ColoredTriangle> {
        //check that all three coordinates are within the bounding box
        // the minpoint is the self.min_point minus self.radius in each direction
        let min_point = self.min_point - Vec3d::new(self.radius, self.radius, self.radius);
        let max_point = self.max_point + Vec3d::new(self.radius, self.radius, self.radius);
        // compute the index of the sphere that contains the point
        let x_coordinate = ((point.x - min_point.x) / (self.radius * 2.0)).floor() as usize;
        let y_coordinate = ((point.y - min_point.y) / (self.radius * 2.0)).floor() as usize;
        let z_coordinate = ((point.z - min_point.z) / (self.radius * 2.0)).floor() as usize;
        let spheres_per_y = ((max_point.y - min_point.y) / (self.radius * 2.0)).floor() as usize;
        let spheres_per_z = ((max_point.z - min_point.z) / (self.radius * 2.0)).floor() as usize;
        let index = x_coordinate * spheres_per_y * spheres_per_z
            + y_coordinate * spheres_per_z
            + z_coordinate;
        if point.x >= min_point.x
            && point.x <= max_point.x
            && point.y >= min_point.y
            && point.y <= max_point.y
            && point.z >= min_point.z
            && point.z <= max_point.z
            && index < self.spheres.len()
        {
            return self.spheres[index].clone();
        }

        // calculate the center of the sphere at the point
        let center = Vec3d::new(
            min_point.x + (x_coordinate as f64 + 0.5) * self.radius * 2.0,
            min_point.y + (y_coordinate as f64 + 0.5) * self.radius * 2.0,
            min_point.z + (z_coordinate as f64 + 0.5) * self.radius * 2.0,
        );
        let radius_times_sqrt_3 = self.radius * (3.0_f64).sqrt();
        WrappedContainingSphere::new(ContainingSphere::new(center, radius_times_sqrt_3))
    }

    fn deduce_hit_color(
        &self,
        ray: Ray,
        containing_sphere: &WrappedContainingSphere<ColoredTriangle>,
    ) -> Option<[f32; 3]> {
        let mut hit_color = None;
        let mut min_dist = f64::INFINITY;
        for tri in &containing_sphere.sphere.contained_shapes {
            if let Some(dist) = ray_triangle_intersect(&ray, &tri.vertices) {
                if dist < min_dist {
                    min_dist = dist;
                    hit_color = Some(tri.color);
                }
            }
        }
        hit_color
    }

    fn get_all_eight_corners_of_min_max_point(&self) -> [Vec3d; 8] {
        let min_point = self.min_point;
        let max_point = self.max_point;
        [
            Vec3d::new(min_point.x, min_point.y, min_point.z),
            Vec3d::new(min_point.x, min_point.y, max_point.z),
            Vec3d::new(min_point.x, max_point.y, min_point.z),
            Vec3d::new(min_point.x, max_point.y, max_point.z),
            Vec3d::new(max_point.x, min_point.y, min_point.z),
            Vec3d::new(max_point.x, min_point.y, max_point.z),
            Vec3d::new(max_point.x, max_point.y, min_point.z),
            Vec3d::new(max_point.x, max_point.y, max_point.z),
        ]
    }
}

#[derive(Clone)]
struct BallLine {
    _balls: Vec<ColoredSphere>,
    _center_x: f64,
    _center_y: f64,
    _spacing: f64,
}

impl BallLine {
    fn new(num_balls: usize, center_x: f64, center_y: f64) -> Self {
        let mut balls = Vec::new();
        let spacing = 2.0;
        for i in 0..num_balls {
            let center = Vec3d::new(
                center_x + (i as f64 - (num_balls as f64 - 1.0) / 2.0) * spacing,
                center_y,
                0.0,
            );
            let radius = 0.5;
            let color = [0.5, 0.5, 0.5];
            balls.push(ColoredSphere::new(center, radius, color));
        }
        BallLine {
            _balls: balls,
            _center_x: center_x,
            _center_y: center_y,
            _spacing: spacing,
        }
    }
}

#[derive(Clone)]
pub struct UniverseObject2D {
    _balls: Vec<BallLine>,
    spheres: Vec<WrappedContainingSphere<ColoredSphere>>,
    grid: Grid<ColoredSphere>,
    _coordinate_system: CoordinateSystem3D,
    min_point: Vec3d,
    max_point: Vec3d,
    radius: f64,
}

impl UniverseObject2D {
    pub fn new(num_balls_x: usize, num_balls_y: usize) -> Self {
        let mut balls = Vec::new();

        // vec_1 is (0.5, sqrt(3)/2.0, 0.0)
        let vec_1 = Vec3d::new(0.5, (3.0_f64).sqrt() / 2.0, 0.0);
        // vec_2 is (-0.5, sqrt(3)/2.0, 0.0)
        let vec_2 = Vec3d::new(-0.5, (3.0_f64).sqrt() / 2.0, 0.0);
        for i in 0..num_balls_y {
            let center = Vec3d::new(0.0, 0.0, 0.0);
            let num_1 = i / 2 + i % 2;
            let num_2 = i / 2;
            let offset = vec_1 * (num_1 as f64) + vec_2 * (num_2 as f64);
            let new_center = center + offset;
            let ball_line = BallLine::new(num_balls_x, new_center.x, new_center.y);
            balls.push(ball_line);
        }
        let mut obj = UniverseObject2D {
            _balls: balls,
            spheres: Vec::new(),
            grid: Grid::<ColoredSphere>::new(Vec3d::new(0.0, 0.0, 0.0), 0.0),
            _coordinate_system: CoordinateSystem3D::standard(),
            min_point: Vec3d::new(0.0, 0.0, 0.0),
            max_point: Vec3d::new(0.0, 0.0, 0.0),
            radius: 0.0,
        };
        obj.deduce_bounding_box_and_position_spheres();
        obj
    }

    fn deduce_bounding_box_and_position_spheres(&mut self) {
        self.radius = 2.0_f64.sqrt();
        for ball_line in &self._balls {
            for ball in &ball_line._balls {
                let min_point = ball.sphere.center
                    - Vec3d::new(ball.sphere.radius, ball.sphere.radius, ball.sphere.radius);
                let max_point = ball.sphere.center
                    + Vec3d::new(ball.sphere.radius, ball.sphere.radius, ball.sphere.radius);
                self.min_point.x = self.min_point.x.min(min_point.x);
                self.min_point.y = self.min_point.y.min(min_point.y);
                self.min_point.z = self.min_point.z.min(min_point.z);
                self.max_point.x = self.max_point.x.max(max_point.x);
                self.max_point.y = self.max_point.y.max(max_point.y);
                self.max_point.z = self.max_point.z.max(max_point.z);
            }
        }

        // add containing spheres with the radius of offset
        let mut current_point = self.min_point;
        while current_point.x <= self.max_point.x {
            while current_point.y <= self.max_point.y {
                while current_point.z <= self.max_point.z {
                    self.spheres
                        .push(self.grid.get_containing_sphere(current_point));
                    current_point.z += self.radius * 2.0;
                }
                current_point.y += self.radius * 2.0;
            }
            current_point.x += self.radius * 2.0;
        }

        // add the balls to the containing spheres
        for ball_line in &self._balls {
            for ball in &ball_line._balls {
                let point = ball.sphere.center;
                let containing_sphere = self.get_sphere(point);
                containing_sphere.add_shape(ball.clone());
            }
        }
    }
}

impl ObjectLike<ColoredSphere> for UniverseObject2D {
    fn radius(&self) -> f64 {
        self.radius
    }

    fn get_sphere(&self, point: Vec3d) -> WrappedContainingSphere<ColoredSphere> {
        let diameter = self.radius * 2.0;
        let index_x = ((point.x - self.min_point.x) / diameter).floor() as usize;
        let index_y = ((point.y - self.min_point.y) / diameter).floor() as usize;
        let index_z = ((point.z - self.min_point.z) / diameter).floor() as usize;
        let spheres_per_y = ((self.max_point.y - self.min_point.y) / diameter).floor() as usize;
        let spheres_per_z = ((self.max_point.z - self.min_point.z) / diameter).floor() as usize;
        let index = index_x * spheres_per_y * spheres_per_z + index_y * spheres_per_z + index_z;
        if index < self.spheres.len() {
            return self.spheres[index].clone();
        }
        // calculate the center of the containing sphere
        self.grid.get_containing_sphere(point)
    }

    fn deduce_hit_color(
        &self,
        ray: Ray,
        containing_sphere: &WrappedContainingSphere<ColoredSphere>,
    ) -> Option<[f32; 3]> {
        let mut hit_color = None;
        let mut min_dist = f64::INFINITY;
        for ball in &containing_sphere.sphere.contained_shapes {
            // Ray-sphere intersection
            let oc = ray.origin - ball.sphere.center;
            let a = Vec3d::dot(&ray.direction, &ray.direction);
            let b = 2.0 * Vec3d::dot(&oc, &ray.direction);
            let c = Vec3d::dot(&oc, &oc) - ball.sphere.radius * ball.sphere.radius;
            let discriminant = b * b - 4.0 * a * c;
            if discriminant > 0.0 {
                let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
                let t2 = (-b + discriminant.sqrt()) / (2.0 * a);
                let t = if t1 > 1e-6 { t1 } else { t2 };
                if t > 1e-6 && t < min_dist {
                    min_dist = t;
                    hit_color = Some(ball.color);
                }
            }
        }
        hit_color
    }

    fn get_all_eight_corners_of_min_max_point(&self) -> [Vec3d; 8] {
        let min_point = self.min_point;
        let max_point = self.max_point;
        [
            Vec3d::new(min_point.x, min_point.y, min_point.z),
            Vec3d::new(min_point.x, min_point.y, max_point.z),
            Vec3d::new(min_point.x, max_point.y, min_point.z),
            Vec3d::new(min_point.x, max_point.y, max_point.z),
            Vec3d::new(max_point.x, min_point.y, min_point.z),
            Vec3d::new(max_point.x, min_point.y, max_point.z),
            Vec3d::new(max_point.x, max_point.y, min_point.z),
            Vec3d::new(max_point.x, max_point.y, max_point.z),
        ]
    }
}

pub struct Voxel {
    pub position: Vec3d,
    pub color: [f32; 3],
}

pub struct MedicalObject3D {
    _coordinate_system: CoordinateSystem3D,
    min_point: Vec3d,
    max_point: Vec3d,
    radius: f64,
    cell_length: f64,
    containing_spheres: Vec<WrappedContainingSphere<ColoredSphere>>,
    grid: Grid<ColoredSphere>,
    color_threshold: f32,
}

impl MedicalObject3D {
    pub fn new(voxels: Vec<Voxel>, radius: f64, cell_length: f64, color_threshold: f32) -> Self {
        let mut obj = MedicalObject3D {
            _coordinate_system: CoordinateSystem3D::standard(),
            min_point: Vec3d::new(0.0, 0.0, 0.0),
            max_point: Vec3d::new(0.0, 0.0, 0.0),
            radius,
            cell_length,
            containing_spheres: Vec::new(),
            grid: Grid::<ColoredSphere>::new(Vec3d::new(0.0, 0.0, 0.0), 0.0),
            color_threshold,
        };
        obj.deduce_bounding_box_and_position_spheres(voxels, radius);
        obj
    }

    fn deduce_bounding_box_and_position_spheres(&mut self, voxels: Vec<Voxel>, radius: f64) {
        // deduce bounding box
        let mut min_point = Vec3d::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max_point = Vec3d::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for voxel in &voxels {
            min_point.x = min_point.x.min(voxel.position.x);
            min_point.y = min_point.y.min(voxel.position.y);
            min_point.z = min_point.z.min(voxel.position.z);
            max_point.x = max_point.x.max(voxel.position.x);
            max_point.y = max_point.y.max(voxel.position.y);
            max_point.z = max_point.z.max(voxel.position.z);
        }

        self.min_point = min_point;
        self.max_point = max_point;
        self.grid = Grid::<ColoredSphere>::new(min_point, self.cell_length);

        // position spheres
        let diameter = self.cell_length * (3.0_f64).sqrt();
        let mut current_point = min_point;
        while current_point.x <= max_point.x {
            while current_point.y <= max_point.y {
                while current_point.z <= max_point.z {
                    // Here you would add a sphere at current_point with the calculated radius
                    self.containing_spheres
                        .push(self.grid.get_containing_sphere(current_point));
                    current_point.z += diameter; // Move to the next position in z
                }
                current_point.y += diameter; // Move to the next position in y
                current_point.z = min_point.z; // Reset z to min
            }
            current_point.x += diameter; // Move to the next position in x
            current_point.y = min_point.y; // Reset y to min
            current_point.z = min_point.z; // Reset z to min
        }

        // add voxels to containing spheres
        for voxel in voxels {
            let point = voxel.position;
            let containing_sphere = self.get_sphere(point);
            containing_sphere.add_shape(ColoredSphere::new(point, radius, voxel.color));
        }
    }
}

impl ObjectLike<ColoredSphere> for MedicalObject3D {
    fn radius(&self) -> f64 {
        self.radius
    }

    fn get_sphere(&self, point: Vec3d) -> WrappedContainingSphere<ColoredSphere> {
        let diameter = self.cell_length * (3.0_f64).sqrt() * 2.0;
        let index_x = ((point.x - self.min_point.x) / diameter).floor() as usize;
        let index_y = ((point.y - self.min_point.y) / diameter).floor() as usize;
        let index_z = ((point.z - self.min_point.z) / diameter).floor() as usize;
        let spheres_per_y = ((self.max_point.y - self.min_point.y) / diameter).floor() as usize;
        let spheres_per_z = ((self.max_point.z - self.min_point.z) / diameter).floor() as usize;
        let index = index_x * spheres_per_y * spheres_per_z + index_y * spheres_per_z + index_z;
        if index < self.containing_spheres.len() {
            return self.containing_spheres[index].clone();
        }
        self.grid.get_containing_sphere(point)
    }

    fn deduce_hit_color(
        &self,
        ray: Ray,
        containing_sphere: &WrappedContainingSphere<ColoredSphere>,
    ) -> Option<[f32; 3]> {
        let mut hit_color = None;
        let mut min_dist = f64::INFINITY;
        for ball in &containing_sphere.sphere.contained_shapes {
            let color = ball.color;
            let color_in_gray = 0.299 * color[0] + 0.587 * color[1] + 0.114 * color[2];
            if color_in_gray < self.color_threshold {
                continue;
            }
            // Ray-sphere intersection
            let oc = ray.origin - ball.sphere.center;
            let a = Vec3d::dot(&ray.direction, &ray.direction);
            let b = 2.0 * Vec3d::dot(&oc, &ray.direction);
            let c = Vec3d::dot(&oc, &oc) - ball.sphere.radius * ball.sphere.radius;
            let discriminant = b * b - 4.0 * a * c;
            if discriminant > 0.0 {
                let t1 = (-b - discriminant.sqrt()) / (2.0 * a);
                let t2 = (-b + discriminant.sqrt()) / (2.0 * a);
                let t = if t1 > 1e-6 { t1 } else { t2 };
                if t > 1e-6 && t < min_dist {
                    min_dist = t;
                    hit_color = Some(ball.color);
                }
            }
        }
        hit_color
    }

    fn get_all_eight_corners_of_min_max_point(&self) -> [Vec3d; 8] {
        let min_point = self.min_point;
        let max_point = self.max_point;
        [
            Vec3d::new(min_point.x, min_point.y, min_point.z),
            Vec3d::new(min_point.x, min_point.y, max_point.z),
            Vec3d::new(min_point.x, max_point.y, min_point.z),
            Vec3d::new(min_point.x, max_point.y, max_point.z),
            Vec3d::new(max_point.x, min_point.y, min_point.z),
            Vec3d::new(max_point.x, min_point.y, max_point.z),
            Vec3d::new(max_point.x, max_point.y, min_point.z),
            Vec3d::new(max_point.x, max_point.y, max_point.z),
        ]
    }
}

struct RayIntersector<Shape: HasVertices + Clone + 'static, O: ObjectLike<Shape>> {
    _marker: std::marker::PhantomData<O>,
    _shape_marker: std::marker::PhantomData<Shape>,
}

impl<Shape: HasVertices + Clone + 'static, O: ObjectLike<Shape>> RayIntersector<Shape, O> {
    fn new() -> Self {
        RayIntersector {
            _marker: std::marker::PhantomData,
            _shape_marker: std::marker::PhantomData,
        }
    }

    pub fn deduce_pixel_color_fast(
        &self,
        object: &O,
        ray: Ray,
        mut cached_sphere: Option<WrappedContainingSphere<Shape>>,
        mut cached_distance: Option<f64>,
    ) -> (
        [f32; 3],
        Option<WrappedContainingSphere<Shape>>,
        Option<f64>,
    ) {
        let hit_color = None;
        let all_eight_distances = object
            .get_all_eight_corners_of_min_max_point()
            .iter()
            .map(|corner| (*corner - ray.origin).length())
            .collect::<Vec<f64>>();

        let is_same_sphere = match &cached_sphere {
            Some(sphere) => {
                let point_in_sphere = ray.origin + ray.direction * cached_distance.unwrap_or(0.0);
                let sphere_at_point = object.get_sphere(point_in_sphere);

                let cached = sphere;
                // check that the two centers are close enough to a certain min distance
                (sphere_at_point.sphere.sphere.center - cached.sphere.sphere.center).length() < 1e-6
            }
            None => false,
        };

        if is_same_sphere {
            let sphere = cached_sphere.as_ref().unwrap();
            let hit_color = object.deduce_hit_color(ray, sphere);
            if let Some(hit_color) = hit_color {
                return (hit_color, cached_sphere, cached_distance);
            } else {
                return ([0.0, 0.0, 0.0], cached_sphere, cached_distance);
            }
        }

        let mut current_point = ray.origin;
        while hit_color.is_none() {
            let sphere = object.get_sphere(current_point);
            if sphere.sphere.contained_shapes.is_empty() {
                let current_distance = (current_point - ray.origin).length();
                // check that if the current point is beyond the max distance to the bounding box corners
                if all_eight_distances.iter().all(|&d| current_distance > d) {
                    cached_sphere = None;
                    cached_distance = Some(current_distance);
                    break;
                }
                current_point = current_point + ray.direction * (object.radius() * 2.0);
                continue;
            };

            let hit_color = object.deduce_hit_color(ray.clone(), &sphere);
            let current_distance = (current_point - ray.origin).length();
            if hit_color.is_some() {
                cached_sphere = Some(sphere);
                cached_distance = Some(current_distance);
                break;
            }
            // check that if the current point is beyond the max distance to the bounding box corners
            if all_eight_distances.iter().all(|&d| current_distance > d) {
                cached_sphere = None;
                cached_distance = Some(current_distance);
                break;
            }

            current_point = current_point + ray.direction * (object.radius() * 2.0);
        }

        if let Some(hit_color) = hit_color {
            (hit_color, cached_sphere, cached_distance)
        } else {
            ([0.0, 0.0, 0.0], cached_sphere, cached_distance)
        }
    }
}

/// @brief Represents the scene containing triangles
/// @param triangles The triangles in the scene
/// @param coordinate_system The coordinate system (standard cartesian)
pub struct Scene<Shape: HasVertices + Clone + 'static, O: ObjectLike<Shape>> {
    _shape_marker: std::marker::PhantomData<Shape>,
    object: O,
    _coordinate_system: CoordinateSystem3D,
    camera: Camera,
    pixel_width: u32,
    pixel_height: u32,
}

impl<Shape: HasVertices + Clone + 'static, O: ObjectLike<Shape>> Scene<Shape, O> {
    /// @brief Creates a new scene with the standard cartesian coordinate system and a camera looking at the origin
    /// @return Scene
    pub fn new(obj: O, camera: Camera) -> Self {
        Scene {
            _shape_marker: std::marker::PhantomData,
            object: obj,
            camera,
            _coordinate_system: CoordinateSystem3D::standard(),
            pixel_width: 800,
            pixel_height: 450,
        }
    }

    pub fn object(&self) -> &O {
        &self.object
    }

    /// @brief Renders the scene to a bitmap by brute-force ray-triangle intersection
    /// @param filename Output file name for the bitmap
    /// @return None
    pub fn take_picture(&self, filename: &str) {
        use image::{Rgb, RgbImage};
        let mut img = RgbImage::new(self.pixel_width, self.pixel_height);

        let tiles = self.deduce_tiles(2, 2);
        let all_colors = tiles
            .iter()
            .map(|tile| {
                let colors = self.deduce_pixel_colors_fast(tile.clone());
                let mut index = 0;
                let mut color_results = Vec::new();
                for _ in tile.start_y..tile.end_y {
                    for _ in tile.start_x..tile.end_x {
                        let hit_color = colors[index];
                        index += 1;
                        let rgb = Rgb([
                            (hit_color[0] * 255.0) as u8,
                            (hit_color[1] * 255.0) as u8,
                            (hit_color[2] * 255.0) as u8,
                        ]);
                        color_results.push(rgb);
                    }
                }
                color_results
            })
            .collect::<Vec<_>>();
        for (tile, color_results) in tiles.iter().zip(all_colors.iter()) {
            let mut index = 0;
            for y in tile.start_y..tile.end_y {
                for x in tile.start_x..tile.end_x {
                    img.put_pixel(x, y, color_results[index]);
                    index += 1;
                }
            }
        }
        img.save(filename).expect("Failed to save image");
    }

    /// @brief Returns a reference to the camera
    /// @return Reference to Camera
    pub fn _camera(&self) -> &Camera {
        &self.camera
    }

    pub fn deduce_pixel_colors_fast(&self, tile: Tile) -> Vec<[f32; 3]> {
        let mut colors = Vec::new();
        let mut cached_sphere: Option<WrappedContainingSphere<Shape>> = None;
        let mut cached_distance = None;
        for y in tile.start_y..tile.end_y {
            for x in tile.start_x..tile.end_x {
                let u = (x as f32 + 0.5) / self.pixel_width as f32;
                let v = (y as f32 + 0.5) / self.pixel_height as f32;
                let ray = self.camera.generate_ray(u, v);
                let ray_intersector = RayIntersector::<Shape, O>::new();
                let (color, new_cached_sphere, new_cached_distance) = ray_intersector
                    .deduce_pixel_color_fast(
                        &self.object,
                        ray,
                        cached_sphere.clone(),
                        cached_distance,
                    );
                cached_sphere = new_cached_sphere;
                cached_distance = new_cached_distance;
                colors.push(color);
            }
        }
        colors
    }

    fn deduce_tiles(&self, tiles_x: u32, tiles_y: u32) -> Vec<Tile> {
        let mut tiles = Vec::new();
        let tile_width = self.pixel_width / tiles_x;
        let tile_height = self.pixel_height / tiles_y;

        for ty in 0..tiles_y {
            for tx in 0..tiles_x {
                let start_x = tx * tile_width;
                let end_x = if tx == tiles_x - 1 {
                    self.pixel_width
                } else {
                    (tx + 1) * tile_width
                };
                let start_y = ty * tile_height;
                let end_y = if ty == tiles_y - 1 {
                    self.pixel_height
                } else {
                    (ty + 1) * tile_height
                };
                tiles.push(Tile {
                    start_x,
                    end_x,
                    start_y,
                    end_y,
                });
            }
        }
        tiles
    }
}
