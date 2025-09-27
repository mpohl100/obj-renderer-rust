
use std::env;
use obj::{Obj, load_obj};
use mtl::{Mtl, load_mtl};
mod scene;
use scene::{Scene, ColoredTriangle};

/// @brief Main entry point. Loads .obj and .mtl files, builds scene.
/// @param args Command line arguments: <obj_file> <mtl_file>
/// @return Exit code
fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() != 3 {
        eprintln!("Usage: {} <obj_file> <mtl_file>", args[0]);
        std::process::exit(1);
    }
    let obj_path = &args[1];
    let mtl_path = &args[2];

    // Load OBJ
    let obj: Obj = match load_obj(obj_path) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("Failed to load OBJ: {}", e);
            std::process::exit(1);
        }
    };

    // Load MTL
    let mtl: Mtl = match load_mtl(mtl_path) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("Failed to load MTL: {}", e);
            std::process::exit(1);
        }
    };

    // Build scene
    let mut scene = Scene::new();
    for object in obj.objects.iter() {
        for group in object.groups.iter() {
            for poly in group.polys.iter() {
                if poly.0.len() == 3 {
                    // Get vertices
                    let v_idx = |i| poly.0[i].0;
                    let v = [
                        obj.vertices[v_idx(0)],
                        obj.vertices[v_idx(1)],
                        obj.vertices[v_idx(2)],
                    ];
                    // Convert to Point3D
                    let vertices = [
                        math3d::Point3D::new(v[0][0], v[0][1], v[0][2]),
                        math3d::Point3D::new(v[1][0], v[1][1], v[1][2]),
                        math3d::Point3D::new(v[2][0], v[2][1], v[2][2]),
                    ];
                    // Get color from MTL
                    let color = mtl.materials.get(&group.material).map(|mat| mat.kd).unwrap_or([1.0, 1.0, 1.0]);
                    scene.triangles.push(ColoredTriangle { vertices, color });
                }
            }
        }
    }
    println!("Loaded scene with {} triangles.", scene.triangles.len());
}
