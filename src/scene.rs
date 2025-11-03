use clap::Error;
use rs_math3d::FloatVector;
use rs_math3d::Vector;
use rs_math3d::{CrossProduct, Vec3d};

use std::sync::Arc;

use crate::camera::Camera;
use crate::camera::Ray;

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
    if u < 0.0 || u > 1.0 {
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

struct Sphere {
    center: Vec3d,
    radius: f64,
}

struct ContainingSphere {
    sphere: Sphere,
    contained_triangles: Vec<ColoredTriangle>, // Indices of triangles contained within this sphere
}

impl ContainingSphere {
    fn new(center: Vec3d, radius: f64) -> Self {
        ContainingSphere {
            sphere: Sphere { center, radius },
            contained_triangles: Vec::new(),
        }
    }

    fn contains(&self, triangle: &ColoredTriangle) -> bool {
        for &v in &triangle.vertices {
            if (v - self.sphere.center).length() < self.sphere.radius {
                return true;
            }
        }
        false
    }

    fn add_triangle(&mut self, triangle: ColoredTriangle) {
        if self.contains(&triangle) {
            self.contained_triangles.push(triangle);
        }
    }
}

#[derive(Clone)]
struct WrappedContainingSphere {
    sphere: Arc<ContainingSphere>,
}

impl WrappedContainingSphere {
    fn new(sphere: ContainingSphere) -> Self {
        WrappedContainingSphere { sphere: Arc::new(sphere) }
    }

    fn contains(&self, triangle: &ColoredTriangle) -> bool {
        self.sphere.contains(triangle) 
    }

    fn add_triangle(&self, triangle: ColoredTriangle) {
        Arc::get_mut(&mut self.sphere.clone()).unwrap().add_triangle(triangle);
    }
}

pub struct Tile {
    pub start_x: u32,
    pub end_x: u32,
    pub start_y: u32,
    pub end_y: u32,
}

/// @brief Represents the scene containing triangles
/// @param triangles The triangles in the scene
/// @param coordinate_system The coordinate system (standard cartesian)
pub struct Scene {
    triangles: Vec<ColoredTriangle>,
    spheres: Vec<WrappedContainingSphere>,
    camera: Camera,
    pixel_width: u32,
    pixel_height: u32,
    min_point: Vec3d,
    max_point: Vec3d,
    radius: f64,
}

impl Scene {
    /// @brief Creates a new scene with the standard cartesian coordinate system and a camera looking at the origin
    /// @return Scene
    pub fn new() -> Self {
        let camera = Camera::new(
            Vec3d::new(0.0, 0.0, 5.0), // position
            Vec3d::new(0.0, 1.0, 0.0), // up
            60.0,                      // fov
            16.0 / 9.0,                // aspect ratio
            0.1,                       // near
            100.0,                     // far
        );
        Scene {
            triangles: Vec::new(),
            spheres: Vec::new(),
            camera,
            pixel_width: 800,
            pixel_height: 450,
            min_point: Vec3d::new(0.0, 0.0, 0.0),
            max_point: Vec3d::new(0.0, 0.0, 0.0),
            radius: 1.0,
        }
    }
    /// @brief Renders the scene to a bitmap by brute-force ray-triangle intersection
    /// @param filename Output file name for the bitmap
    /// @return None
    pub fn take_picture(&self, filename: &str) {
        use image::{Rgb, RgbImage};
        let mut img = RgbImage::new(self.pixel_width, self.pixel_height);
        for y in 0..self.pixel_height {
            for x in 0..self.pixel_width {
                // Compute normalized device coordinates
                let u = (x as f32 + 0.5) / self.pixel_width as f32;
                let v = (y as f32 + 0.5) / self.pixel_height as f32;
                // Generate ray from camera through pixel
                let ray = self.camera.generate_ray(u, v);
                // Brute-force intersection
                let mut hit_color = [0.0, 0.0, 0.0];
                let mut min_dist = f64::INFINITY;
                for tri in &self.triangles {
                    if let Some(dist) = ray_triangle_intersect(&ray, &tri.vertices) {
                        if dist < min_dist {
                            min_dist = dist;
                            hit_color = tri.color;
                        }
                    }
                }
                let rgb = Rgb([
                    (hit_color[0] * 255.0) as u8,
                    (hit_color[1] * 255.0) as u8,
                    (hit_color[2] * 255.0) as u8,
                ]);
                img.put_pixel(x, y, rgb);
            }
        }
        img.save(filename).expect("Failed to save image");
    }

    /// @brief Adds a colored triangle to the scene
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

        let mut current_point = self.min_point;
        while current_point.x <= self.max_point.x {
            while current_point.y <= self.max_point.y {
                while current_point.z <= self.max_point.z {
                    // Here you would add a sphere at current_point with the calculated radius
                    current_point.z += self.radius * 2.0; // Move to the next position in z
                    self.add_sphere(current_point, self.radius);
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
                sphere.add_triangle(tri.clone());
            }
        }
    }

    /// @brief Returns a reference to the triangles in the scene
    /// @return Reference to Vec<ColoredTriangle>
    pub fn triangles(&self) -> &Vec<ColoredTriangle> {
        &self.triangles
    }

    /// @brief Returns a reference to the camera
    /// @return Reference to Camera
    pub fn camera(&self) -> &Camera {
        &self.camera
    }

    pub fn deduce_pixel_colors_fast(&self, tile: Tile) -> Vec<[f32; 3]> {
        let mut colors = Vec::new();
        let all_eight_corners_of_min_max_point = [
            Vec3d::new(self.min_point.x, self.min_point.y, self.min_point.z),
            Vec3d::new(self.min_point.x, self.min_point.y, self.max_point.z),
            Vec3d::new(self.min_point.x, self.max_point.y, self.min_point.z),
            Vec3d::new(self.min_point.x, self.max_point.y, self.max_point.z),
            Vec3d::new(self.max_point.x, self.min_point.y, self.min_point.z),
            Vec3d::new(self.max_point.x, self.min_point.y, self.max_point.z),
            Vec3d::new(self.max_point.x, self.max_point.y, self.min_point.z),
            Vec3d::new(self.max_point.x, self.max_point.y, self.max_point.z),
        ];
        let mut cached_sphere: Option<WrappedContainingSphere> = None;
        let mut cached_distance = None;
        for y in tile.start_y..tile.end_y {
            for x in tile.start_x..tile.end_x {
                let u = (x as f32 + 0.5) / self.pixel_width as f32;
                let v = (y as f32 + 0.5) / self.pixel_height as f32;
                let ray = self.camera.generate_ray(u, v);
                let mut hit_color = None;
                let mut min_dist = f64::INFINITY;
                let all_eight_distances = all_eight_corners_of_min_max_point
                    .iter()
                    .map(|corner| (*corner - ray.origin).length())
                    .collect::<Vec<f64>>();

                // probe whether we hit the same sphere as last time
                let is_same_sphere = match &cached_sphere {
                    Some(sphere) => {
                        let point_in_sphere =
                            ray.origin + ray.direction * cached_distance.unwrap_or(0.0);
                        let sphere_at_point = self.get_sphere(point_in_sphere);
                        match sphere_at_point {
                            Some(s) => {
                                let cached = cached_sphere.as_ref().unwrap();
                                // check that the two centers are close enough to a certain min distance
                                (s.sphere.sphere.center - cached.sphere.sphere.center).length() < 1e-6
                            },
                            None => false,
                        }
                    }
                    None => false,
                };

                if is_same_sphere {
                    let sphere = cached_sphere.as_ref().unwrap();
                    let hit_color = self.deduce_hit_color(ray, sphere);
                    if let Some(hit_color) = hit_color {
                        colors.push(hit_color);
                        continue;
                    } else {
                        colors.push([0.0, 0.0, 0.0]);
                        continue;
                    }
                }

                let mut current_point = ray.origin;
                while hit_color.is_none() {
                    let sphere = match self.get_sphere(current_point) {
                        Some(s) => s,
                        None => {
                            current_point = current_point + ray.direction * (self.radius * 2.0);
                            continue;
                        }
                    };
                    let hit_color = self.deduce_hit_color(ray.clone(), &sphere);

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

                    current_point = current_point + ray.direction * (self.radius * 2.0);
                }

                if let Some(hit_color) = hit_color {
                    colors.push(hit_color);
                } else {
                    colors.push([0.0, 0.0, 0.0]);
                }
            }
        }
        colors
    }

    fn deduce_hit_color(&self, ray: Ray, containing_sphere: &WrappedContainingSphere) -> Option<[f32; 3]> {
        let mut hit_color = None;
        let mut min_dist = f64::INFINITY;
        for tri in &containing_sphere.sphere.contained_triangles {
            if let Some(dist) = ray_triangle_intersect(&ray, &tri.vertices) {
                if dist < min_dist {
                    min_dist = dist;
                    hit_color = Some(tri.color);
                }
            }
        }
        hit_color
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

    fn add_sphere(&mut self, center: Vec3d, radius: f64) {
        self.spheres.push(WrappedContainingSphere::new(ContainingSphere::new(center, radius)));
    }

    fn get_sphere(&self, point: Vec3d) -> Option<WrappedContainingSphere> {
        //check that all three coordinates are within the bounding box
        if point.x >= self.min_point.x
            && point.x <= self.max_point.x
            && point.y >= self.min_point.y
            && point.y <= self.max_point.y
            && point.z >= self.min_point.z
            && point.z <= self.max_point.z
        {
            // compute the index of the sphere that contains the point
            let x_coordinate =
                ((point.x - self.min_point.x) / (self.radius * 2.0)).floor() as usize;
            let y_coordinate =
                ((point.y - self.min_point.y) / (self.radius * 2.0)).floor() as usize;
            let z_coordinate =
                ((point.z - self.min_point.z) / (self.radius * 2.0)).floor() as usize;
            let spheres_per_y =
                ((self.max_point.y - self.min_point.y) / (self.radius * 2.0)).floor() as usize;
            let spheres_per_z =
                ((self.max_point.z - self.min_point.z) / (self.radius * 2.0)).floor() as usize;
            let index = x_coordinate * spheres_per_y * spheres_per_z
                + y_coordinate * spheres_per_z
                + z_coordinate;
            if index < self.spheres.len() {
                return Some(self.spheres[index].clone());
            }
        }
        None
    }
}
