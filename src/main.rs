use obj_renderer_rust::camera::Camera;
use obj_renderer_rust::scene::{ColoredTriangle, Object3D, Scene};

use clap::Parser;
use wavefront_obj::obj::parse;

/// @brief Command line arguments for obj-renderer
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the OBJ file
    #[arg(long = "obj-file")]
    obj_file: String,

    /// Path to the MTL file
    #[arg(long = "mtl-file")]
    mtl_file: String,

    /// Output image filename
    #[arg(long = "output-file", default_value = "output.png")]
    output_file: String,
}

use rs_math3d::Vector;
/// @brief Main entry point. Loads .obj and .mtl files, builds scene.
/// @param args Command line arguments: --obj-file <obj_file> --mtl-file <mtl_file>
/// @return Exit code
use rs_math3d::{CrossProduct, Vec3d};

/// @brief Checks if three points make a convex corner (CCW)
fn is_convex(prev: &Vec3d, curr: &Vec3d, next: &Vec3d) -> bool {
    let v1 = *curr - *prev;
    let v2 = *next - *curr;
    let cross = CrossProduct::cross(&v1, &v2);
    cross.z > 0.0
}

/// @brief Checks if a point is inside a triangle (barycentric method)
fn point_in_triangle(p: &Vec3d, tri: [&Vec3d; 3]) -> bool {
    let v0 = *tri[2] - *tri[0];
    let v1 = *tri[1] - *tri[0];
    let v2 = *p - *tri[0];
    let dot00 = Vec3d::dot(&v0, &v0);
    let dot01 = Vec3d::dot(&v0, &v1);
    let dot02 = Vec3d::dot(&v0, &v2);
    let dot11 = Vec3d::dot(&v1, &v1);
    let dot12 = Vec3d::dot(&v1, &v2);
    let denom = dot00 * dot11 - dot01 * dot01;
    if denom.abs() < 1e-8 {
        return false;
    }
    let inv_denom = 1.0 / denom;
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
    (u >= 0.0) && (v >= 0.0) && (u + v < 1.0)
}

/// @brief Triangulate a polygon using ear clipping
fn triangulate(polygon: &[Vec3d]) -> Vec<[Vec3d; 3]> {
    let mut triangles = Vec::new();
    let mut vertices: Vec<Vec3d> = polygon.to_vec();
    while vertices.len() > 3 {
        let n = vertices.len();
        let mut ear_found = false;
        for i in 0..n {
            let prev = &vertices[(i + n - 1) % n];
            let curr = &vertices[i];
            let next = &vertices[(i + 1) % n];
            if is_convex(prev, curr, next) {
                let tri = [*prev, *curr, *next];
                if !vertices.iter().enumerate().any(|(j, v)| {
                    j != (i + n - 1) % n
                        && j != i
                        && j != (i + 1) % n
                        && point_in_triangle(v, [&tri[0], &tri[1], &tri[2]])
                }) {
                    triangles.push(tri);
                    vertices.remove(i);
                    ear_found = true;
                    break;
                }
            }
        }
        if !ear_found {
            panic!("No ear found – polygon may be malformed");
        }
    }
    triangles.push([vertices[0], vertices[1], vertices[2]]);
    triangles
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let dome = parse(args.obj_file).expect("Failed to parse OBJ file");

    let mut obj = Object3D::new();

    // parse all the polygons of the dome
    for object in dome.objects {
        let mut polygon = Vec::new();
        for vertex in object.vertices {
            polygon.push(Vec3d::new(vertex.x, vertex.y, vertex.z));
        }
        let triangles = triangulate(&polygon);
        for tri in triangles {
            obj.add_colored_triangle(ColoredTriangle {
                vertices: tri,
                color: [0.8, 0.8, 0.8], // gray color for now
            });
        }
    }

    obj.position_spheres();
    let camera = Camera::new(
        Vec3d::new(0.0, 0.0, 5.0), // position
        Vec3d::new(0.0, 0.0, 0.0), // look_at
        Vec3d::new(0.0, 1.0, 0.0), // up
        60.0,                      // fov
        16.0 / 9.0,                // aspect ratio
    );
    let scene = Scene::new(obj, camera);
    println!(
        "Loaded scene with {} triangles.",
        scene.object().triangles().len()
    );

    scene.take_picture(&args.output_file);

    Ok(())
}
