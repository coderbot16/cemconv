extern crate cem;
extern crate cgmath;
extern crate structopt;
#[macro_use]
extern crate structopt_derive;
extern crate wavefront_obj;

use wavefront_obj::obj::{self, Object, Primitive, VTNIndex};
use std::fs::File;
use std::collections::HashMap;
use std::io::{self, Read, Write};
use cem::{ModelHeader, v2, V2, Scene, Model, Encode};
use cgmath::{Point2, Point3, Vector3, Matrix4, Deg, InnerSpace};

#[derive(StructOpt, Debug)]
struct Opt {
	#[structopt(short = "i", long = "input", help = "Input file to convert, default is stdout")]
	input: Option<String>,
	#[structopt(short = "g", long = "iformat", help = "Format to use for the input")]
	input_format: Option<String>,
	#[structopt(short = "f", long = "format", help = "Format to use as the output")]
	format: String,
	#[structopt(short = "n", long = "frame", help = "Frame number in the CEM file to extract")]
	frame_index: Option<usize>,
	#[structopt(help = "Output file, default is stdout")]
	output: Option<String>
}

enum Format {
	Cem { version: (u16, u16), frame_index: usize },
	Obj
}

impl Format {
	fn parse(format: &str, frame_index: Option<usize>) -> Option<Self> {
		let frame_index = frame_index.unwrap_or(0);

		Some(match format {
			"cem1.3" => Format::Cem { version: (1 ,3), frame_index },
			"cem2" => Format::Cem { version: (2, 0), frame_index },
			"cem" => Format::Cem { version: (2, 0), frame_index },
			"ssmf" => Format::Cem { version: (2, 0), frame_index },
			"obj" => Format::Obj,
			_ => return None
		})
	}
}

fn main() {
	use structopt::StructOpt;

	let opt = Opt::from_args();

	let format = match Format::parse(&opt.format, opt.frame_index) {
		Some(format) => format,
		None => {
			eprintln!("Unrecognized output format {:?}", opt.format);
			return;
		}
	};

	let input_format = match Format::parse(opt.input_format.as_ref().map(|s| s as &str).unwrap_or(""), opt.frame_index) {
		Some(format) => format,
		None => Format::Cem { version: (2, 0), frame_index: opt.frame_index.unwrap_or(0) }
	};

	let stdin = io::stdin();
	let stdout = io::stdout();

	let result = match (opt.input, opt.output) {
		(None, None) => convert (
			stdin.lock(),
			stdout.lock(),
			input_format,
			format
		),
		(None, Some(path)) => convert (
			stdin.lock(),
			match File::create(&path) {
				Ok(file) => file,
				Err(e) => {
					eprintln!("error: failed to create the output file at {} ({})", path, e);
					return
				}
			},
			input_format,
			format
		),
		(Some(path), None) => convert (
			match File::open(&path) {
				Ok(file) => file,
				Err(e) => {
					eprintln!("error: failed to open the input file at {} ({})", path, e);
					return
				}
			},
			stdout.lock(),
			input_format,
			format
		),
		(Some(input), Some(output)) => convert (
			match File::open(&input) {
				Ok(file) => file,
				Err(e) => {
					eprintln!("error: failed open the input file at {} ({})", input, e);
					return
				}
			},
			match File::create(&output) {
				Ok(file) => file,
				Err(e) => {
					eprintln!("error: failed to create the output file at {} ({})", output, e);
					return
				}
			},
			input_format,
			format
		)
	};

	if let Err(e) = result {
		eprintln!("error: conversion failed: {}", e);
	}
}

fn convert<I, O>(mut i: I, mut o: O, input_format: Format, format: Format) -> io::Result<()> where I: Read, O: Write {
	match (input_format, format) {
		(Format::Obj, Format::Cem { version: (2, 0), frame_index: _ }) => {
			let mut buffer = String::new();
			i.read_to_string(&mut buffer)?;

			let obj = obj::parse(buffer).map_err(
				|parse| io::Error::new(io::ErrorKind::InvalidData, format!("Error in OBJ file on line {}: {}", parse.line_number, parse.message))
			)?;

			let model = obj_to_cem(&obj.objects[0]);

			Scene::root(model).write(&mut o)
		},
		(Format::Cem { version: (2, 0), frame_index: _ }, Format::Cem { version: (2, 0), frame_index: _ }) => {
			let header = ModelHeader::read(&mut i)?;

			if header == V2::HEADER {
				Scene::<V2>::read_without_header(&mut i)?.write(&mut o)
			} else {
				unimplemented!("Cannon rewrite non-CEMv2 files yet.")
			}
		},
		(Format::Cem { version: (_, _), frame_index }, Format::Obj) => {
			let header = ModelHeader::read(&mut i)?;

			if header == V2::HEADER {
				let scene = Scene::<V2>::read_without_header(&mut i)?;

				if frame_index >= scene.model.frames.len() {
					return Err(io::Error::new(io::ErrorKind::InvalidInput, format!("Tried to extract frame index {} from a CEM file that only has {} frames", frame_index, scene.model.frames.len())));
				}

				let buffer = cem2_to_obj(scene.model, frame_index);

				o.write_all(buffer.as_bytes())
			} else {
				unimplemented!("Cannon convert non-CEMv2 files to OBJ yet.")
			}
		},
		_ => unimplemented!()
	}
}

fn obj_to_cem(i: &Object) -> V2 {
	let mut triangles = Vec::new();
	let mut vertices = Vec::new();

	let transformation = Matrix4::from_angle_x(Deg(90.0));

	{
		let mut vertex_associations = HashMap::new();

		let mut resolve_index = |v: VTNIndex| {
			*vertex_associations.entry(v).or_insert_with(|| {
				let index = vertices.len();

				let position = i.vertices[v.0];
				let texture = v.1.map(|index| i.tex_vertices[index]).unwrap_or(obj::TVertex { u: 0.0, v: 0.0, w: 0.0 });
				let normal = v.2.map(|index| i.normals[index]).unwrap_or(obj::Vertex { x: 1.0, y: 0.0, z: 0.0 });

				let normal = Vector3 { x: normal.x as f32, y: normal.y as f32, z: normal.z as f32 };
				let position = Point3 { x: position.x as f32, y: position.y as f32, z: position.z as f32 };

				let normal = (transformation * normal.normalize().extend(0.0)).truncate();
				let position = Point3::from_homogeneous(transformation * position.to_homogeneous());

				vertices.push(v2::Vertex {
					position,
					normal,
					texture: Point2 { x: texture.u as f32, y: texture.v as f32 },
				});

				index
			})
		};

		for geometry in &i.geometry {
			for primitive in geometry.shapes.iter().map(|shape| shape.primitive) {
				match primitive {
					Primitive::Triangle(v0, v1, v2) => {
						triangles.push((
							resolve_index(v0) as u32,
							resolve_index(v1) as u32,
							resolve_index(v2) as u32
						));
					},
					_ => () // Skip lines and points, not supported.
				}
			}
		}
	}

	// Create the model

	let mut center_builder = ::cem::collider::CenterBuilder::begin();

	for vertex in &vertices {
		center_builder.update(vertex.position);
	}

	let center = center_builder.build();

	V2 {
		center,
		materials: vec![v2::Material {
			name: "".to_string(),
			texture: 0,
			triangles: vec![
				v2::TriangleSelection {
					offset: 0,
					len: triangles.len() as u32
				}
			],
			vertex_offset: 0,
			vertex_count: vertices.len() as u32,
			texture_name: "".to_string()
		}],
		lod_levels: vec![
			triangles
		],
		tag_points: vec![],
		frames: vec![
			v2::Frame::from_vertices(vertices, vec![], center)
		]
	}
}

fn cem2_to_obj(cem: V2, frame_index: usize) -> String {
	use std::fmt::Write;

	let triangle_data = &cem.lod_levels[0];
	let frame = &cem.frames[frame_index];

	let mut string = String::new();

	let transformation = Matrix4::from_angle_x(Deg(-90.0));

	for &v2::Vertex { position, normal, texture } in frame.vertices.iter() {

		let normal = (transformation * normal.normalize().extend(0.0)).truncate();
		let position = Point3::from_homogeneous(transformation * position.to_homogeneous());

		writeln!(string, "v {} {} {}", position.x, position.y, position.z).unwrap();
		writeln!(string, "vn {} {} {}", normal.x, normal.y, normal.z).unwrap();
		writeln!(string, "vt {} {}", texture.x, texture.y).unwrap();
	}

	for &v2::Material { ref name, texture, ref triangles, vertex_offset, vertex_count: _vertex_count, ref texture_name } in &cem.materials {
		let triangle_slice = triangles[0];

		writeln!(string, "# name: {}, texture: {}, texture_name: {}", name, texture, texture_name).unwrap();

		for index in 0..triangle_slice.len {
			let index = index + triangle_slice.offset;
			let triangle = &triangle_data[index as usize];

			let indices = (
				vertex_offset + triangle.0 + 1,
				vertex_offset + triangle.1 + 1,
				vertex_offset + triangle.2 + 1
			);

			writeln!(string, "f {}/{}/{} {}/{}/{} {}/{}/{}", indices.0, indices.0, indices.0, indices.1, indices.1, indices.1, indices.2, indices.2, indices.2).unwrap();
		}
	}

	string
}