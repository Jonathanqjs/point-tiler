use std::{
    collections::{HashMap, HashSet},
    fs::{self, File},
    io::{BufReader, BufWriter, Read, Write},
    path::{Path, PathBuf},
};

use pcd_core::pointcloud::point::{Point, PointCloud};
use pcd_exporter::gltf::GlbOptions;
use pcd_parser::{
    parser::Extension,
    reader::{PointReader, csv::CsvPointReader, las::LasPointReader, ply::PlyPointReader},
};
use serde_json::json;
use tempfile::tempdir;

use crate::ConvertOptions;

const MAX_POINTS_PER_TILE: u64 = 100_000;
type TileCoord = (u8, u32, u32);

#[derive(Debug, Clone, Copy, PartialEq)]
struct Bounds {
    min: [f64; 3],
    max: [f64; 3],
}

impl Bounds {
    fn empty() -> Self {
        Self {
            min: [f64::MAX; 3],
            max: [f64::MIN; 3],
        }
    }

    fn include_point(&mut self, point: &Point) {
        self.min[0] = self.min[0].min(point.x);
        self.min[1] = self.min[1].min(point.y);
        self.min[2] = self.min[2].min(point.z);
        self.max[0] = self.max[0].max(point.x);
        self.max[1] = self.max[1].max(point.y);
        self.max[2] = self.max[2].max(point.z);
    }

    fn include_bounds(&mut self, other: Self) {
        for axis in 0..3 {
            self.min[axis] = self.min[axis].min(other.min[axis]);
            self.max[axis] = self.max[axis].max(other.max[axis]);
        }
    }

    fn center(self) -> [f64; 3] {
        std::array::from_fn(|axis| (self.min[axis] + self.max[axis]) * 0.5)
    }

    fn spans(self) -> [f64; 3] {
        std::array::from_fn(|axis| (self.max[axis] - self.min[axis]).max(0.0))
    }

    fn box_volume(self) -> [f64; 12] {
        let center = self.center();
        let span = self.spans();
        [
            center[0],
            center[1],
            center[2],
            span[0] * 0.5,
            0.0,
            0.0,
            0.0,
            span[1] * 0.5,
            0.0,
            0.0,
            0.0,
            span[2] * 0.5,
        ]
    }
}

#[derive(Debug, Clone, Copy)]
struct CartesianGrid {
    min_x: f64,
    min_y: f64,
    span: f64,
}

impl CartesianGrid {
    fn from_bounds(bounds: Bounds) -> Self {
        Self {
            min_x: bounds.min[0],
            min_y: bounds.min[1],
            span: (bounds.max[0] - bounds.min[0])
                .max(bounds.max[1] - bounds.min[1])
                .max(f64::EPSILON),
        }
    }

    fn tile(self, depth: u8, point: &Point) -> TileCoord {
        let side = 1u64 << depth;
        let index = |value: f64, min: f64| {
            (((value - min) / self.span * side as f64).floor() as i64).clamp(0, side as i64 - 1)
                as u32
        };
        (
            depth,
            index(point.x, self.min_x),
            index(point.y, self.min_y),
        )
    }
}

fn parent((z, x, y): TileCoord) -> TileCoord {
    (z - 1, x / 2, y / 2)
}

fn open_reader(paths: Vec<PathBuf>, extension: Extension) -> std::io::Result<Box<dyn PointReader>> {
    match extension {
        Extension::Las | Extension::Laz => Ok(Box::new(LasPointReader::new(paths)?)),
        Extension::Csv | Extension::Txt => Ok(Box::new(CsvPointReader::new(paths)?)),
        Extension::Ply => Ok(Box::new(PlyPointReader::new(paths)?)),
    }
}

fn scan_bounds(paths: &[PathBuf], extension: Extension) -> std::io::Result<(Bounds, u64)> {
    let mut reader = open_reader(paths.to_vec(), extension)?;
    let mut bounds = Bounds::empty();
    let mut count = 0;
    while let Some(point) = reader.next_point()? {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "Cartesian point coordinates must be finite",
            ));
        }
        bounds.include_point(&point);
        count += 1;
    }
    if count == 0 {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            "input point cloud is empty",
        ));
    }
    Ok((bounds, count))
}

fn estimated_depth(bounds: Bounds, count: u64, maximum: u8) -> u8 {
    let spans = bounds.spans();
    let longest = spans[0].max(spans[1]).max(f64::EPSILON);
    let target_size = if spans[0].min(spans[1]) <= longest * 1e-9 {
        longest * MAX_POINTS_PER_TILE as f64 / count as f64
    } else {
        (spans[0] * spans[1] * MAX_POINTS_PER_TILE as f64 / count as f64).sqrt()
    }
    .max(f64::EPSILON);
    ((CartesianGrid::from_bounds(bounds).span / target_size)
        .log2()
        .ceil()
        .max(0.0) as u8)
        .min(maximum)
}

fn histogram(
    paths: &[PathBuf],
    extension: Extension,
    grid: CartesianGrid,
    depth: u8,
) -> std::io::Result<HashMap<TileCoord, u64>> {
    let mut reader = open_reader(paths.to_vec(), extension)?;
    let mut counts = HashMap::new();
    while let Some(point) = reader.next_point()? {
        *counts.entry(grid.tile(depth, &point)).or_insert(0) += 1;
    }
    Ok(counts)
}

fn adaptive_leaves(counts: &HashMap<TileCoord, u64>, depth: u8) -> HashSet<TileCoord> {
    let mut all_counts = counts.clone();
    for z in (1..=depth).rev() {
        let level = all_counts
            .iter()
            .filter(|((tile_z, _, _), _)| *tile_z == z)
            .map(|(tile, count)| (*tile, *count))
            .collect::<Vec<_>>();
        for (tile, count) in level {
            *all_counts.entry(parent(tile)).or_insert(0) += count;
        }
    }
    fn visit(
        tile: TileCoord,
        depth: u8,
        counts: &HashMap<TileCoord, u64>,
        leaves: &mut HashSet<TileCoord>,
    ) {
        let count = counts.get(&tile).copied().unwrap_or(0);
        if count == 0 {
            return;
        }
        if count <= MAX_POINTS_PER_TILE || tile.0 == depth {
            leaves.insert(tile);
            return;
        }
        let (z, x, y) = tile;
        for dx in 0..2 {
            for dy in 0..2 {
                visit((z + 1, x * 2 + dx, y * 2 + dy), depth, counts, leaves);
            }
        }
    }
    let mut leaves = HashSet::new();
    visit((0, 0, 0), depth, &all_counts, &mut leaves);
    leaves
}

fn leaf_for(
    point: &Point,
    grid: CartesianGrid,
    depth: u8,
    leaves: &HashSet<TileCoord>,
) -> TileCoord {
    let mut tile = grid.tile(depth, point);
    loop {
        if leaves.contains(&tile) {
            return tile;
        }
        tile = parent(tile);
    }
}

fn write_points(path: &Path, points: &[Point]) -> std::io::Result<()> {
    fs::create_dir_all(path.parent().unwrap())?;
    let mut writer = BufWriter::new(File::create(path)?);
    writer.write_all(&bitcode::encode(points))?;
    writer.flush()
}

fn read_points(path: &Path) -> std::io::Result<Vec<Point>> {
    let mut bytes = Vec::new();
    BufReader::new(File::open(path)?).read_to_end(&mut bytes)?;
    bitcode::decode(&bytes).map_err(|error| {
        std::io::Error::new(
            std::io::ErrorKind::InvalidData,
            format!("bitcode decode failed: {error}"),
        )
    })
}

fn tile_path(root: &Path, (z, x, y): TileCoord) -> PathBuf {
    root.join(format!("{z}/{x}/{y}.bin"))
}

fn write_leaf_runs(
    paths: &[PathBuf],
    extension: Extension,
    grid: CartesianGrid,
    depth: u8,
    leaves: &HashSet<TileCoord>,
    run_root: &Path,
    max_memory_mb: usize,
) -> std::io::Result<()> {
    let max_points = ((max_memory_mb.max(1) * 1024 * 1024) / std::mem::size_of::<Point>())
        .clamp(10_000, 1_000_000);
    let mut reader = open_reader(paths.to_vec(), extension)?;
    let mut buckets = HashMap::<TileCoord, Vec<Point>>::new();
    let mut buffered = 0;
    let mut run_index = 0;
    let flush = |buckets: &mut HashMap<TileCoord, Vec<Point>>, run_index: usize| {
        for (tile, points) in buckets.drain() {
            let (z, x, y) = tile;
            write_points(
                &run_root.join(format!("{z}/{x}/{y}/run-{run_index}.bin")),
                &points,
            )?;
        }
        Ok::<_, std::io::Error>(())
    };
    while let Some(point) = reader.next_point()? {
        buckets
            .entry(leaf_for(&point, grid, depth, leaves))
            .or_default()
            .push(point);
        buffered += 1;
        if buffered >= max_points {
            flush(&mut buckets, run_index)?;
            run_index += 1;
            buffered = 0;
        }
    }
    flush(&mut buckets, run_index)
}

fn materialize_leaves(
    run_root: &Path,
    tile_root: &Path,
    leaves: &HashSet<TileCoord>,
) -> std::io::Result<()> {
    for &(z, x, y) in leaves {
        let pattern = run_root.join(format!("{z}/{x}/{y}/*.bin"));
        let mut points = Vec::new();
        for path in glob::glob(pattern.to_str().unwrap())
            .map_err(|error| std::io::Error::other(error.to_string()))?
            .filter_map(Result::ok)
        {
            points.extend(read_points(&path)?);
        }
        write_points(&tile_path(tile_root, (z, x, y)), &points)?;
    }
    Ok(())
}

fn decimate(points: Vec<Point>, voxel_size: f64, disabled: bool) -> Vec<Point> {
    if disabled || points.is_empty() || voxel_size <= 0.0 {
        return points;
    }
    let mut selected = HashMap::<(i64, i64, i64), Point>::new();
    for point in points {
        let key = (
            (point.x / voxel_size).floor() as i64,
            (point.y / voxel_size).floor() as i64,
            (point.z / voxel_size).floor() as i64,
        );
        selected.entry(key).or_insert(point);
    }
    selected.into_values().collect()
}

fn parse_tile_path(path: &Path) -> TileCoord {
    let x = path
        .parent()
        .unwrap()
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let z = path
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .file_name()
        .unwrap()
        .to_str()
        .unwrap()
        .parse()
        .unwrap();
    let y = path.file_stem().unwrap().to_str().unwrap().parse().unwrap();
    (z, x, y)
}

fn aggregate_parents(
    tile_root: &Path,
    grid: CartesianGrid,
    maximum_depth: u8,
    disabled: bool,
) -> std::io::Result<()> {
    for child_z in (1..=maximum_depth).rev() {
        let pattern = tile_root.join(format!("{child_z}/**/*.bin"));
        let mut parents = HashMap::<TileCoord, Vec<PathBuf>>::new();
        for path in glob::glob(pattern.to_str().unwrap())
            .map_err(|error| std::io::Error::other(error.to_string()))?
            .filter_map(Result::ok)
        {
            parents
                .entry(parent(parse_tile_path(&path)))
                .or_default()
                .push(path);
        }
        for (tile, children) in parents {
            let mut points = Vec::new();
            for child in children {
                points.extend(read_points(&child)?);
            }
            let cell_span = grid.span / (1u64 << tile.0) as f64;
            write_points(
                &tile_path(tile_root, tile),
                &decimate(points, cell_span / 64.0, disabled),
            )?;
        }
    }
    Ok(())
}

#[derive(Debug)]
struct TileInfo {
    coord: TileCoord,
    uri: String,
    bounds: Bounds,
}

#[derive(Default)]
struct TreeNode {
    content: Option<TileInfo>,
    children: HashMap<(u32, u32), TreeNode>,
}

impl TreeNode {
    fn insert(&mut self, info: TileInfo) {
        let (z, x, y) = info.coord;
        let mut node = self;
        for level in 1..=z {
            let shift = z - level;
            node = node.children.entry((x >> shift, y >> shift)).or_default();
        }
        node.content = Some(info);
    }

    fn into_tile(self) -> (cesiumtiles::tileset::Tile, Bounds) {
        let mut bounds = self.content.as_ref().map(|info| info.bounds);
        let mut entries = self.children.into_iter().collect::<Vec<_>>();
        entries.sort_by_key(|(key, _)| *key);
        let mut children = Vec::new();
        for (_, child) in entries {
            let (child, child_bounds) = child.into_tile();
            if let Some(current) = &mut bounds {
                current.include_bounds(child_bounds);
            } else {
                bounds = Some(child_bounds);
            }
            children.push(child);
        }
        let bounds = bounds.expect("Cartesian tree node must contain points");
        let spans = bounds.spans();
        let leaf = children.is_empty();
        let tile = cesiumtiles::tileset::Tile {
            bounding_volume: cesiumtiles::tileset::BoundingVolume::new_box(bounds.box_volume()),
            geometric_error: if leaf {
                0.0
            } else {
                spans[0].max(spans[1]).max(spans[2])
            },
            refine: Some(cesiumtiles::tileset::Refine::Replace),
            content: self.content.map(|info| cesiumtiles::tileset::Content {
                uri: info.uri,
                ..Default::default()
            }),
            children: (!children.is_empty()).then_some(children),
            ..Default::default()
        };
        (tile, bounds)
    }
}

fn export_tiles(
    tile_root: &Path,
    output: &Path,
    maximum_depth: u8,
    options: &GlbOptions,
) -> std::io::Result<Vec<TileInfo>> {
    let mut infos = Vec::new();
    for z in 0..=maximum_depth {
        let pattern = tile_root.join(format!("{z}/**/*.bin"));
        for path in glob::glob(pattern.to_str().unwrap())
            .map_err(|error| std::io::Error::other(error.to_string()))?
            .filter_map(Result::ok)
        {
            let (_, x, y) = parse_tile_path(&path);
            let cloud = PointCloud::new(read_points(&path)?, 0);
            let bounds = Bounds {
                min: cloud.metadata.bounding_volume.min,
                max: cloud.metadata.bounding_volume.max,
            };
            let uri = format!("{z}/{x}/{y}.glb");
            let destination = output.join(&uri);
            fs::create_dir_all(destination.parent().unwrap())?;
            let glb = pcd_exporter::gltf::generate_glb_with_options(cloud, options)
                .map_err(|error| std::io::Error::other(error.to_string()))?;
            glb.to_writer_with_alignment(BufWriter::new(File::create(destination)?), 8)?;
            infos.push(TileInfo {
                coord: (z, x, y),
                uri,
                bounds,
            });
        }
    }
    Ok(infos)
}

pub(crate) fn convert(
    input_files: Vec<PathBuf>,
    extension: Extension,
    args: &ConvertOptions,
    output: &Path,
) -> std::io::Result<()> {
    let (bounds, point_count) = scan_bounds(&input_files, extension)?;
    let grid = CartesianGrid::from_bounds(bounds);
    let mut depth = estimated_depth(bounds, point_count, args.max);
    let counts = loop {
        let counts = histogram(&input_files, extension, grid, depth)?;
        if depth == args.max || counts.values().all(|count| *count <= MAX_POINTS_PER_TILE) {
            break counts;
        }
        depth += 1;
    };
    let leaves = adaptive_leaves(&counts, depth);
    log::info!(
        "Cartesian bounds {:?}..{:?}, points {}, depth {}, leaves {}",
        bounds.min,
        bounds.max,
        point_count,
        depth,
        leaves.len()
    );

    let temporary = tempdir()?;
    let run_root = temporary.path().join("runs");
    let tile_root = temporary.path().join("tiles");
    write_leaf_runs(
        &input_files,
        extension,
        grid,
        depth,
        &leaves,
        &run_root,
        args.max_memory_mb,
    )?;
    materialize_leaves(&run_root, &tile_root, &leaves)?;
    aggregate_parents(&tile_root, grid, depth, args.disable_decimation)?;

    let options = GlbOptions {
        quantize: args.quantize,
        meshopt: args.meshopt,
        gzip_compress: false,
    };
    let infos = export_tiles(&tile_root, output, depth, &options)?;
    let mut tree = TreeNode::default();
    for info in infos {
        tree.insert(info);
    }
    let (root, _) = tree.into_tile();
    let root_error = root.geometric_error;
    let tileset = cesiumtiles::tileset::Tileset {
        asset: cesiumtiles::tileset::Asset {
            version: "1.1".into(),
            extras: Some(json!({
                "coordinateSystem": "CARTESIAN_METERS",
                "bounds": [bounds.min[0], bounds.min[1], bounds.min[2], bounds.max[0], bounds.max[1], bounds.max[2]],
                "pointCount": point_count
            })),
            ..Default::default()
        },
        root,
        geometric_error: root_error,
        ..Default::default()
    };
    let mut value = serde_json::to_value(tileset).map_err(std::io::Error::other)?;
    value["asset"]["gltfUpAxis"] = json!("Z");
    fs::write(
        output.join("tileset.json"),
        serde_json::to_string_pretty(&value).map_err(std::io::Error::other)?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use pcd_core::pointcloud::point::{Color, PointAttributes};

    fn point(x: f64, y: f64, z: f64) -> Point {
        Point {
            x,
            y,
            z,
            color: Color::default(),
            attributes: PointAttributes {
                intensity: None,
                return_number: None,
                classification: None,
                scanner_channel: None,
                scan_angle: None,
                user_data: None,
                point_source_id: None,
                gps_time: None,
            },
        }
    }

    #[test]
    fn adaptive_tree_splits_busy_nodes_and_preserves_sparse_nodes() {
        let counts = HashMap::from([((2, 0, 0), 80_000), ((2, 1, 0), 80_000), ((2, 3, 3), 10)]);
        let leaves = adaptive_leaves(&counts, 2);
        assert!(leaves.contains(&(2, 0, 0)));
        assert!(leaves.contains(&(2, 1, 0)));
        assert!(leaves.contains(&(1, 1, 1)));
    }

    #[test]
    fn grid_clamps_max_bound_to_last_cell() {
        let grid = CartesianGrid::from_bounds(Bounds {
            min: [0.0; 3],
            max: [8.0, 2.0, 1.0],
        });
        assert_eq!(grid.tile(3, &point(8.0, 2.0, 1.0)), (3, 7, 2));
    }

    #[test]
    fn depth_estimate_handles_degenerate_line_bounds() {
        let bounds = Bounds {
            min: [0.0, 0.0, 0.0],
            max: [100.0, 0.0, 1.0],
        };
        assert_eq!(estimated_depth(bounds, 4_000_000, 20), 6);
    }

    #[test]
    fn conversion_writes_cartesian_box_and_metadata_without_reprojection() {
        let temporary = tempdir().unwrap();
        let input = temporary.path().join("points.ply");
        let output = temporary.path().join("tiles");
        let mut file = File::create(&input).unwrap();
        writeln!(
            file,
            "ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n10 20 30 255 0 0\n12 24 36 0 255 0"
        )
        .unwrap();
        drop(file);

        let args = ConvertOptions {
            input_reference: crate::InputReference::CartesianMeters,
            output: output.to_string_lossy().into_owned(),
            max: 4,
            max_memory_mb: 1,
            ..Default::default()
        };
        fs::create_dir_all(&output).unwrap();
        convert(vec![input], Extension::Ply, &args, &output).unwrap();

        let value: serde_json::Value =
            serde_json::from_slice(&fs::read(output.join("tileset.json")).unwrap()).unwrap();
        assert_eq!(value["asset"]["gltfUpAxis"], "Z");
        assert_eq!(
            value["asset"]["extras"]["coordinateSystem"],
            "CARTESIAN_METERS"
        );
        assert_eq!(
            value["asset"]["extras"]["bounds"],
            json!([10.0, 20.0, 30.0, 12.0, 24.0, 36.0])
        );
        assert!(value["root"]["boundingVolume"]["box"].is_array());
        assert!(value["root"]["boundingVolume"]["region"].is_null());
    }
}
