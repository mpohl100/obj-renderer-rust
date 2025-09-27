mod camera;
mod scene;
use scene::{ColoredTriangle, Scene};

use clap::Parser;
use std::fs::File;
use std::io::BufReader;
use obj::{load_obj, Obj};

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
}

/// @brief Main entry point. Loads .obj and .mtl files, builds scene.
/// @param args Command line arguments: --obj-file <obj_file> --mtl-file <mtl_file>
/// @return Exit code
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    // Load OBJ from file

    let input = BufReader::new(File::open(args.obj_file)?);
    let dome: Obj = load_obj(input)?;

    // TODO: Parse MTL file using obj.materials if available
    let mut scene = Scene::new();

    println!("Loaded scene with {} triangles.", scene.triangles().len());
    Ok(())
}
