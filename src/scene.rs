use rs_math3d::{CrossProduct, Vec3d};
use rs_math3d::Vector;

use crate::camera::Camera;

/// @brief Checks intersection of a ray with a triangle using Möller–Trumbore algorithm
/// @param ray The ray
/// @param vertices The triangle vertices
/// @return Some(distance) if hit, None otherwise
pub fn ray_triangle_intersect(ray: &crate::camera::Ray, vertices: &[rs_math3d::Vec3d; 3]) -> Option<f64> {
    let v0 = vertices[0];
    let v1 = vertices[1];
    let v2 = vertices[2];
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = CrossProduct::cross(&ray.direction, &edge2);
    let a = Vec3d::dot(&edge1, &h);
    if a.abs() < 1e-6 { return None; }
    let f = 1.0 / a;
    let s = ray.origin - v0;
    let u = f * rs_math3d::Vec3d::dot(&s, &h);
    if u < 0.0 || u > 1.0 { return None; }
    let q = rs_math3d::Vec3d::cross(&s, &edge1);
    let v = f * rs_math3d::Vec3d::dot(&ray.direction, &q);
    if v < 0.0 || u + v > 1.0 { return None; }
    let t = f * rs_math3d::Vec3d::dot(&edge2, &q);
    if t > 1e-6 { Some(t) } else { None }
}

/// @brief Represents a colored triangle in 3D space
/// @param vertices The three vertices of the triangle
/// @param color The color of the triangle
pub struct ColoredTriangle {
    pub vertices: [Vec3d; 3],
    pub color: [f32; 3], // RGB
}

/// @brief Represents the scene containing triangles
/// @param triangles The triangles in the scene
/// @param coordinate_system The coordinate system (standard cartesian)
pub struct Scene {
    triangles: Vec<ColoredTriangle>,
    camera: Camera,
    pixel_width: u32,
    pixel_height: u32,
}

impl Scene {
    /// @brief Creates a new scene with the standard cartesian coordinate system and a camera looking at the origin
    /// @return Scene
    pub fn new() -> Self {
        let camera = Camera::new(
            Vec3d::new(0.0, 0.0, 5.0), // position
            Vec3d::new(0.0, 1.0, 0.0),   // up
            60.0,                        // fov
            16.0/9.0,                    // aspect ratio
            0.1,                         // near
            100.0                        // far
        );
        Scene {
            triangles: Vec::new(),
            camera,
            pixel_width: 800,
            pixel_height: 450,
        }
    }
    /// @brief Renders the scene to a bitmap by brute-force ray-triangle intersection
    /// @param filename Output file name for the bitmap
    /// @return None
    pub fn take_picture(&self, filename: &str) {
        use image::{RgbImage, Rgb};
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
}
