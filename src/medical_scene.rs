use obj_renderer_rust::camera::Camera;
use obj_renderer_rust::scene::{Scene, Voxel, MedicalObject3D};

use clap::Parser;

use rs_math3d::Vec3d;

use std::cmp::Ordering;
use std::error::Error;
use std::fs;
use std::path::Path;

use dicom_object::{open_file, DefaultDicomObject};
use dicom_pixeldata::PixelDecoder;

/// @brief Command line arguments for obj-renderer
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Input directory of the DICOM directory containing the medical images
    #[arg(long = "dicom-input-dir")]
    input_dir: String,
    /// Output image filename
    #[arg(long = "output-file", default_value = "output.png")]
    output_file: String,
}

/// @brief Reads the z position of a DICOM slice
/// @param obj The loaded DICOM object
/// @return The slice z position
fn read_slice_z(obj: &DefaultDicomObject) -> Result<f64, Box<dyn Error>> {
    if let Ok(element) = obj.element_by_name("ImagePositionPatient") {
        let raw = element.to_str()?;
        if let Some(z_str) = raw.split('\\').nth(2) {
            return Ok(z_str.parse::<f64>()?);
        }
    }

    if let Ok(element) = obj.element_by_name("SliceLocation") {
        return Ok(element.to_str()?.parse::<f64>()?);
    }

    if let Ok(element) = obj.element_by_name("InstanceNumber") {
        return Ok(element.to_str()?.parse::<f64>()?);
    }

    Err(std::io::Error::new(
        std::io::ErrorKind::InvalidData,
        "missing ImagePositionPatient, SliceLocation, and InstanceNumber",
    )
    .into())
}

/// @brief Loads all DICOM slices in a directory into voxels
/// @param input_dir Directory containing DICOM files
/// @return A vector of voxels where x/y are pixel coordinates and z is the slice position
fn load_dicom_voxels(input_dir: &str) -> Result<Vec<Voxel>, Box<dyn Error>> {
    let mut slices = Vec::new();

    for entry in fs::read_dir(Path::new(input_dir))? {
        let path = entry?.path();
        if !path.is_file() {
            continue;
        }

        let obj = match open_file(&path) {
            Ok(obj) => obj,
            Err(_) => continue,
        };

        let z = read_slice_z(&obj)?;

        let image = obj.decode_pixel_data()?.to_dynamic_image(0)?.to_luma8();
        slices.push((z, image));
    }

    slices.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));

    let mut voxels = Vec::new();

    for (z, image) in slices {
        for (x, y, pixel) in image.enumerate_pixels() {
            let gray = f32::from(pixel[0]) / 255.0;

            voxels.push(Voxel {
                position: Vec3d::new(x as f64, y as f64, z),
                color: [gray, gray, gray],
            });
        }
    }

    Ok(voxels)
}

/// @brief Loads a medical object from a DICOM directory
/// @param input_dir Directory containing DICOM files
/// @param voxel_radius Radius used for each voxel sphere
/// @param cell_length Spatial cell size used by the acceleration structure
/// @param color_threshold Minimum grayscale intensity to consider during rendering
/// @return A fully constructed medical object
fn load_medical_object_from_dicom(
    input_dir: &str,
    voxel_radius: f64,
    cell_length: f64,
    color_threshold: f32,
) -> Result<MedicalObject3D, Box<dyn Error>> {
    let voxels = load_dicom_voxels(input_dir)?;
    Ok(MedicalObject3D::new(
        voxels,
        voxel_radius,
        cell_length,
        color_threshold,
    ))
}

/// @brief Main entry point. Builds a simple medical scene and renders it.
/// @param args Command line arguments: --output-file <output_file>
/// @return Exit code
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let obj = load_medical_object_from_dicom(&args.input_dir, 0.5, 3.0, 0.1)?;
    let camera = Camera::new(
        Vec3d::new(0.0, 0.0, 5.0), // position
        Vec3d::new(0.0, 0.0, 0.0), // look_at
        Vec3d::new(0.0, 1.0, 0.0), // up
        60.0,                      // fov
        16.0 / 9.0,                // aspect ratio
        0.1,                       // near
        100.0,                     // far
    );
    let scene = Scene::new(obj, camera);
    println!("Loaded scene with medical object.");

    scene.take_picture(&args.output_file);

    Ok(())
}
