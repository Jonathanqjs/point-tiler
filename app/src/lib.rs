use std::ffi::OsStr;
use std::fs;
use std::io::ErrorKind;
use std::path::{Path, PathBuf};

use glob::glob;
use pcd_parser::parser::{Extension, get_extension};
use pcd_parser::reader::las::LasPointReader;
use pcd_parser::reader::ply::PlyPointReader;

mod cartesian;
mod epsg;

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
#[serde(tag = "mode", rename_all = "camelCase")]
pub enum InputReference {
    Epsg { code: u16 },
    CartesianMeters,
}

impl Default for InputReference {
    fn default() -> Self {
        Self::Epsg { code: 0 }
    }
}

pub type ProgressCallback = std::sync::Arc<dyn Fn(f32, &str, Option<&str>) + Send + Sync>;

#[derive(Clone, serde::Deserialize)]
#[serde(default)]
pub struct ConvertOptions {
    pub input: Vec<String>,
    pub output: String,
    pub input_reference: InputReference,
    pub output_epsg: u16,
    pub min: u8,
    pub max: u8,
    pub max_memory_mb: usize,
    pub threads: Option<usize>,
    pub quantize: bool,
    pub gzip_compress: bool,
    pub meshopt: bool,
    pub disable_decimation: bool,
    #[serde(skip)]
    pub on_progress: Option<ProgressCallback>,
}

impl std::fmt::Debug for ConvertOptions {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ConvertOptions")
            .field("input", &self.input)
            .field("output", &self.output)
            .field("input_reference", &self.input_reference)
            .field("output_epsg", &self.output_epsg)
            .field("min", &self.min)
            .field("max", &self.max)
            .field("max_memory_mb", &self.max_memory_mb)
            .field("threads", &self.threads)
            .field("quantize", &self.quantize)
            .field("gzip_compress", &self.gzip_compress)
            .field("meshopt", &self.meshopt)
            .field("disable_decimation", &self.disable_decimation)
            .finish()
    }
}

pub fn report_progress(
    options: &ConvertOptions,
    progress: f32,
    stage: &str,
    message: Option<&str>,
) {
    if let Some(ref cb) = options.on_progress {
        cb(progress, stage, message);
    }
}

impl Default for ConvertOptions {
    fn default() -> Self {
        Self {
            input: Vec::new(),
            output: String::new(),
            input_reference: InputReference::default(),
            output_epsg: 4979,
            min: 15,
            max: 18,
            max_memory_mb: 4 * 1024,
            threads: None,
            quantize: false,
            gzip_compress: false,
            meshopt: false,
            disable_decimation: false,
            on_progress: None,
        }
    }
}

pub(crate) const IN_MEMORY_WORKFLOW_MULTIPLIER: u64 = 5;

pub(crate) fn check_and_get_extension(paths: &[PathBuf]) -> Result<Extension, String> {
    if paths.is_empty() {
        return Err("No input files found".to_string());
    }

    let mut extensions = vec![];
    for path in paths.iter() {
        let extension = path.extension().and_then(OsStr::to_str);
        match extension {
            Some(ext) => extensions.push(ext),
            None => return Err("File extension is not found".to_string()),
        }
    }
    extensions.sort();
    extensions.dedup();

    if extensions.len() > 1 {
        return Err("Multiple extensions are not supported".to_string());
    }

    Ok(get_extension(extensions[0]))
}

pub(crate) fn expand_globs(input_patterns: Vec<String>) -> std::io::Result<Vec<PathBuf>> {
    let mut paths = Vec::new();
    for pattern in input_patterns {
        if pattern.contains('*') || pattern.contains('?') || pattern.contains('[') {
            for entry in glob(&pattern)
                .map_err(|e| std::io::Error::new(ErrorKind::InvalidInput, e.to_string()))?
            {
                paths.push(entry.map_err(|e| std::io::Error::other(e.to_string()))?);
            }
        } else {
            paths.push(PathBuf::from(pattern));
        }
    }
    Ok(paths)
}

pub(crate) fn estimate_total_size(paths: &[PathBuf]) -> u64 {
    paths
        .iter()
        .map(|p| p.metadata().map(|m| m.len()).unwrap_or(0))
        .sum()
}

pub(crate) fn estimate_processing_size(paths: &[PathBuf], extension: Extension) -> u64 {
    match extension {
        Extension::Las | Extension::Laz => paths
            .iter()
            .map(LasPointReader::estimate_processing_size)
            .sum(),
        Extension::Csv | Extension::Txt => estimate_total_size(paths),
        Extension::Ply => paths
            .iter()
            .map(PlyPointReader::estimate_processing_size)
            .sum(),
    }
}

pub(crate) fn estimated_in_memory_requirement_bytes(processing_size: u64) -> u64 {
    processing_size.saturating_mul(IN_MEMORY_WORKFLOW_MULTIPLIER)
}

pub(crate) fn should_use_in_memory(processing_size: u64, max_memory_bytes: u64) -> bool {
    estimated_in_memory_requirement_bytes(processing_size) <= max_memory_bytes
}

pub(crate) fn collect_file_sizes(base_path: &Path, files: &mut Vec<(PathBuf, u64)>) -> std::io::Result<()> {
    for entry in fs::read_dir(base_path)? {
        let entry = entry?;
        let path = entry.path();
        let metadata = entry.metadata()?;
        if metadata.is_dir() {
            collect_file_sizes(&path, files)?;
        } else if metadata.is_file() {
            files.push((path, metadata.len()));
        }
    }
    Ok(())
}

pub(crate) fn format_size(bytes: u64) -> String {
    const KIB: f64 = 1024.0;
    const MIB: f64 = 1024.0 * 1024.0;
    const GIB: f64 = 1024.0 * 1024.0 * 1024.0;

    let bytes = bytes as f64;
    if bytes >= GIB {
        format!("{:.2} GiB", bytes / GIB)
    } else if bytes >= MIB {
        format!("{:.2} MiB", bytes / MIB)
    } else if bytes >= KIB {
        format!("{:.2} KiB", bytes / KIB)
    } else {
        format!("{} B", bytes as u64)
    }
}

pub(crate) fn summarize_directory(base_path: &Path) -> std::io::Result<(usize, u64)> {
    let mut files = Vec::new();
    collect_file_sizes(base_path, &mut files)?;
    let total_bytes = files.iter().map(|(_, size)| *size).sum();
    Ok((files.len(), total_bytes))
}

pub(crate) fn log_directory_summary(label: &str, base_path: &Path) {
    match summarize_directory(base_path) {
        Ok((file_count, total_bytes)) => {
            log::info!(
                "{}: {} across {} files",
                label,
                format_size(total_bytes),
                file_count
            );
        }
        Err(e) => {
            log::warn!("Failed to inspect {} at {:?}: {}", label, base_path, e);
        }
    }
}

pub fn convert(args: ConvertOptions) -> std::io::Result<()> {
    if args.input.is_empty() {
        return Err(std::io::Error::new(
            ErrorKind::InvalidInput,
            "at least one input file is required",
        ));
    }
    if args.output.is_empty() {
        return Err(std::io::Error::new(
            ErrorKind::InvalidInput,
            "an output directory is required",
        ));
    }
    if args.min > args.max {
        return Err(std::io::Error::new(
            ErrorKind::InvalidInput,
            "min zoom must not exceed max zoom",
        ));
    }

    let thread_count = args
        .threads
        .filter(|&n| n > 0)
        .unwrap_or(num_cpus::get() * 2);
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(thread_count)
        .build()
        .map_err(|e| std::io::Error::other(e.to_string()))?;

    pool.install(|| convert_with_pool(args, thread_count))
}

fn convert_with_pool(args: ConvertOptions, thread_count: usize) -> std::io::Result<()> {
    log::info!("rayon threads: {}", thread_count);
    log::info!("input files: {:?}", args.input);
    log::info!("output folder: {}", args.output);
    log::info!("input reference: {:?}", args.input_reference);
    log::info!("output EPSG: {}", args.output_epsg);
    log::info!("min zoom: {}", args.min);
    log::info!("max zoom: {}", args.max);
    log::info!("max memory mb: {}", args.max_memory_mb);
    log::info!("threads: {:?}", args.threads);
    log::info!("quantize: {}", args.quantize);
    log::info!("gzip compress: {}", args.gzip_compress);
    log::info!("meshopt: {}", args.meshopt);
    log::info!("disable decimation: {}", args.disable_decimation);

    let start = std::time::Instant::now();

    log::info!("start processing...");
    let input_files = expand_globs(args.input.clone())?;
    if input_files.is_empty() {
        return Err(std::io::Error::new(
            ErrorKind::NotFound,
            "input pattern did not match any files",
        ));
    }
    if let Some(path) = input_files.iter().find(|path| !path.is_file()) {
        return Err(std::io::Error::new(
            ErrorKind::NotFound,
            format!("input file not found: {}", path.display()),
        ));
    }
    log::info!("Expanded input files: {:?}", input_files);

    let output_path = PathBuf::from(&args.output);
    std::fs::create_dir_all(&output_path)?;

    let extension = check_and_get_extension(&input_files).map_err(std::io::Error::other)?;
    let total_size = estimate_total_size(&input_files);
    let processing_size = estimate_processing_size(&input_files, extension);
    let max_memory_bytes = args.max_memory_mb as u64 * 1024 * 1024;
    let estimated_in_memory_requirement = estimated_in_memory_requirement_bytes(processing_size);
    log::info!(
        "input size: {}, estimated processing size: {}, estimated in-memory requirement (x{}): {}, threshold: {}",
        format_size(total_size),
        format_size(processing_size),
        IN_MEMORY_WORKFLOW_MULTIPLIER,
        format_size(estimated_in_memory_requirement),
        format_size(max_memory_bytes)
    );

    if matches!(args.input_reference, InputReference::CartesianMeters) {
        log::info!("Using Cartesian meters workflow");
        cartesian::convert(input_files, extension, &args, &output_path)?;
    } else {
        log::info!("Using EPSG workflow");
        epsg::convert(input_files, extension, &args, &output_path)?;
    }

    log::info!("Elapsed: {:?}", start.elapsed());
    log::info!("Finish processing");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convert_requires_an_input_file() {
        let error = convert(ConvertOptions {
            output: "output".to_string(),
            ..Default::default()
        })
        .unwrap_err();

        assert_eq!(error.kind(), ErrorKind::InvalidInput);
    }

    #[test]
    fn should_use_in_memory_requires_five_times_processing_size() {
        let processing_size = 100;

        assert!(!should_use_in_memory(processing_size, 499));
        assert!(should_use_in_memory(processing_size, 500));
    }
}
