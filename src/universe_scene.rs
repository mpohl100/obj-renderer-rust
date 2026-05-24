use obj_renderer_rust::camera::Camera;
use obj_renderer_rust::scene::{Scene, UniverseObject2D};

use clap::Parser;

use rs_math3d::Vec3d;

/// @brief Command line arguments for obj-renderer
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Output image filename
    #[arg(long = "output-file", default_value = "output.png")]
    output_file: String,
}

/// @brief Main entry point. Builds a simple universe scene and renders it.
/// @param args Command line arguments: --output-file <output_file>
/// @return Exit code
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let obj = UniverseObject2D::new(100, 100);
    let camera = Camera::new(
        Vec3d::new(0.0, 0.0, 5.0), // position
        Vec3d::new(0.0, 0.0, 0.0), // look_at
        Vec3d::new(0.0, 1.0, 0.0), // up
        60.0,                      // fov
        16.0 / 9.0,                // aspect ratio
    );
    let scene = Scene::new(obj, camera);
    println!("Loaded scene with universe.");

    scene.take_picture(&args.output_file);

    Ok(())
}
