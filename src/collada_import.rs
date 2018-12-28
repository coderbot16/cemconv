use cem::{v2, V2, collider};
use cgmath::{Point3, Point2, Vector3, Matrix4, Deg, InnerSpace};
use collada::{Object, Shape, VTNIndex, TVertex, Vertex as NVertex};
use collada::document::ColladaDocument;
use std::collections::HashMap;
use xml::{self, Element};

pub fn convert(document: ColladaDocument) -> V2 {
	let mut objects = HashMap::new();

	for object in document.get_obj_set().expect("No objects in collada document...").objects {
		objects.insert(object.id.clone(), object);
	}

	let ns = document.root_element.ns.as_ref().map(String::as_ref);

	// Find what frames are attached to each piece of geometry
	let morph_links = document.root_element.get_child("library_controllers", ns).map(|controllers| {
		controllers
			.get_children("controller", ns)
			.filter_map(|controller| controller.get_child("morph", ns))
			.filter_map(|morph| {
				if morph.get_attribute("method", None) == Some("RELATIVE") {
					eprintln!("warning[collada]: unsupported morph method RELATIVE, treating it as NORMALIZED...");
				}

				let name = trim_hash(morph.get_attribute("source", None)?).to_owned();

				// Ignore this for now since we don't do anything with preset morph weights.
				//let weight = get_input(ns, morph, "MORPH_WEIGHT").and_then(|input_element| get_input_source(ns, morph, input_element))?;

				let target = get_input(ns, morph.get_child("targets", ns)?, "MORPH_TARGET")
					.and_then(|input_element| get_input_source(ns, morph, input_element))?;


				// TODO: We don't actually obey the standard here.
				// We need to be using the accessor to read the array instead of reading the array directly.
				// Alas, for now this works (at least with Blender).

				let morph_targets = target.get_child("IDREF_array", ns)?
					.children
					.iter()
					.filter_map(|child| if let &xml::Xml::CharacterNode(ref contents) = child { Some(contents) } else { None })
					.find(|_| true)?
					.trim()
					.split_whitespace()
					.map(str::to_owned)
					.collect::<Vec<_>>();

				Some((name, morph_targets))
			})
			.collect::<HashMap<String, Vec<String>>>()
	}).unwrap_or_else(HashMap::new);

	let primary_scene = trim_hash(document.root_element.get_child("scene", ns)
		.expect("Collada document requires a root scene")
		.get_child("instance_visual_scene", ns)
		.expect("Collada document missing root visual scene")
		.get_attribute("url", None)
		.expect("<instance_visual_scene> missing \"url\" attribute"));



	let nodes = document.root_element.get_child("library_visual_scenes", ns)
		.expect("Collada document has to have visual scenes")
		.get_children("visual_scene", ns)
		.find(|child| child.get_attribute("id", None) == Some(primary_scene))
		.expect("The scene named in <instance_visual_scene> does not exist")
		.get_children("node", ns);

	let mut root_geometry = Vec::new();

	for node in nodes {
		if node.get_attribute("type", None) == Some("JOINT") {
			eprintln!("warning[collada]: unsupported node type JOINT, ignoring...");
			continue;
		}

		for element in node.children.iter().filter_map(|child| if let &xml::Xml::ElementNode(ref element) = child { Some(element) } else { None }) {
			match &element.name as &str {
				"asset" => (),
				"lookat" | "matrix" | "rotate" | "scale" | "skew" | "translate" => {
					eprintln!("warning[collada]: transformations on nodes are not supported yet (tried to use transformation type: {})...", element.name);
				},
				"instance_camera" => eprintln!("warning[collada]: Ignoring instance_camera"),
				"instance_controller" => eprintln!("warning[collada]: Ignoring instance_controller"),
				"instance_geometry" => {
					let object_id = if let Some(url) = element.get_attribute("url", None) {
						trim_hash(url)
					} else {
						eprintln!("warning[collada]: degenerate <instance_geometry> is missing a url tag");
						continue;
					};

					root_geometry.push(object_id.to_owned());
				},
				"instance_light" => eprintln!("warning[collada]: Lights are unsupported"),
				"instance_node" => eprintln!("warning[collada]: Ignoring instance_node"),
				"node" => eprintln!("warning[collada]: Nested nodes are unsupported"),
				_ => ()
			}
		}
	}

	// Needed information extracted. Now begin conversion.

	if root_geometry.len() == 0 {
		panic!("No root geometry!");
	} else if root_geometry.len() > 1 {
		eprintln!("warning[collada]: ignoring additional root geometry for now, submodels are not supported yet");
	}

	let root_name = &root_geometry[0];

	let object = objects.get(root_name).expect("geometry library missing root geometry");
	let object_frames = morph_links.get(root_name)
		.map(|vec| vec.iter()
			.map(|name| objects.get(name).expect("geometry library missing geometry frame").clone()).collect::<Vec<Object>>()
		).unwrap_or_else(Vec::new);

	let mut failed_index = None;

	for (index, frame) in object_frames.iter().enumerate() {
		if object.vertices.len() != frame.vertices.len() || object.normals.len() != frame.normals.len() || object.tex_vertices.len() != frame.tex_vertices.len() {
			failed_index = Some(index);
			break;
		}

		let base = &object.geometry;
		let frame = &frame.geometry;

		if base.len() != frame.len() {
			failed_index = Some(index);
			break;
		}

		if let Some(_) = base.iter().zip(frame.iter()).find(|pair| !compare_geometry(&pair.0.shapes, &pair.1.shapes)) {
			failed_index = Some(index);
			break;
		}
	}

	if let Some(failed_index) = failed_index {
		panic!("error[collada]: index {} in the morph target sequence uses different geometry", failed_index);
	}

	let mut associations = Vec::new();
	let mut reverse = HashMap::new();
	let mut triangles = Vec::new();

	// Note: We make the last entry of each vertex component array the zero/invalid entry for missings
	let invalid_texture_index = object.tex_vertices.len();
	let invalid_normal_index = object.normals.len();

	{
		let mut dedup_vertex = |vertex: VTNIndex| {
			let vertex = (
				vertex.0,
				vertex.1.unwrap_or(invalid_texture_index),
				vertex.2.unwrap_or(invalid_normal_index)
			);

			*reverse.entry(vertex).or_insert_with(|| {
				let index = associations.len();

				associations.push(vertex);

				index
			})
		};

		for geometry in &object.geometry {
			for shape in &geometry.shapes {
				match shape {
					&Shape::Triangle(a, b, c) => {
						triangles.push((
							dedup_vertex(a) as u32,
							dedup_vertex(b) as u32,
							dedup_vertex(c) as u32
						));
					},
					_ => () // Lines / points unsupported
				}
			}
		}
	}

	println!("{} triangles with {} flattened vertices (from: {} position, {} tex, {} normal)", triangles.len(), associations.len(), object.vertices.len(), object.tex_vertices.len(), object.normals.len());

	let mut frames = Vec::with_capacity(1 + object_frames.len());

	// TODO: Tag Points
	let (center, frame0) = extract_frame(&object, &associations, vec![]);

	frames.push(frame0);

	for additional_frame in &object_frames {
		frames.push(extract_frame(additional_frame, &associations, vec![]).1);
	}


	v2::V2 {
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
			vertex_count: associations.len() as u32,
			texture_name: "".to_string()
		}],
		lod_levels: vec![
			triangles
		],
		tag_points: vec![],
		frames
	}
}

fn extract_frame(from: &Object, indices: &[(usize, usize, usize)], tag_points: Vec<Point3<f32>>) -> (Point3<f32>, v2::Frame) {
	let transformation = Matrix4::from_angle_x(Deg(90.0));

	let mut vertices = Vec::with_capacity(indices.len());
	let mut center_builder = collider::CenterBuilder::begin();

	for &(position, texture, normal) in indices {
		let position = from.vertices[position];
		let texture = from.tex_vertices.get(texture).unwrap_or(&TVertex { x: 0.0, y: 0.0 });
		let normal = from.normals.get(normal).unwrap_or(&NVertex { x: 1.0, y: 0.0, z: 0.0 });

		let normal = Vector3 { x: normal.x as f32, y: normal.y as f32, z: normal.z as f32 };
		let position = Point3 { x: position.x as f32, y: position.y as f32, z: position.z as f32 };

		let vertex = v2::Vertex {
			position: Point3::from_homogeneous(transformation * position.to_homogeneous()),
			normal: (transformation * normal.normalize().extend(0.0)).truncate(),
			texture: Point2 { x: texture.x as f32, y: 1.0 - texture.y as f32 },
		};

		center_builder.update(vertex.position);
		vertices.push(vertex);
	}

	let center = center_builder.build();

	(center, v2::Frame::from_vertices(vertices, tag_points, center))
}

// Utilities for COLLADA (Mostly taken from private methods in piston_collada)

fn compare_geometry(base: &[Shape], frame: &[Shape]) -> bool {
	if base.len() != frame.len() {
		return false;
	}

	if let Some(_) = base.iter().zip(frame.iter()).find(|pair| !compare_shape(*pair.0, *pair.1)) {
		return false;
	}

	true
}

fn compare_shape(base: Shape, frame: Shape) -> bool {
	match (base, frame) {
		(Shape::Point(a), Shape::Point(b)) => a==b,
		(Shape::Line(a, a1), Shape::Line(b, b1)) => a==b && a1==b1,
		(Shape::Triangle(a, a1, a2), Shape::Triangle(b, b1, b2)) => a==b && a1==b1 && a2==b2,
		_ => false
	}
}

fn trim_hash(name: &str) -> &str {
	if name.starts_with('#') { &name[1..] } else { name }
}

fn get_input<'a>(ns: Option<&'a str>, parent: &'a Element, semantic : &str) -> Option<&'a Element> {
	let mut inputs = parent.get_children("input", ns);
	inputs.find( |i| i.get_attribute("semantic", None) == Some(semantic))
}

fn get_input_source<'a>(ns: Option<&'a str>, parent_element: &'a Element, input_element: &'a Element) -> Option<&'a Element> {
	let source_id = input_element.get_attribute("source", None)?;

	if let Some(element) = parent_element.children.iter()
		.filter_map(|node| { if let &xml::Xml::ElementNode(ref e) = node { Some(e) } else { None } })
		.find(|e| {
			if let Some(id) = e.get_attribute("id", None) {
				let id = "#".to_string() + id;
				id == source_id
			} else {
				false
			}
		})
		{
			if element.name == "source" {
				return Some(element);
			} else {
				let input = element.get_child("input", ns)?;
				return get_input_source(ns, parent_element, input);
			}
		}
	None
}