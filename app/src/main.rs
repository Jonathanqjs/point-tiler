use chrono::Local;
use clap::{ArgGroup, Parser};
use env_logger::Builder;
use log::LevelFilter;
use point_tiler::{ConvertOptions, InputReference, convert};
use std::io::Write;

#[derive(Parser, Debug)]
#[command(
    name = "Point Tiler",
    about = "A tool for converting point cloud data into 3D Tiles",
    group(
        ArgGroup::new("input_reference")
            .required(true)
            .args(["input_epsg", "cartesian_meters"])
    )
)]
struct Cli {
    #[arg(short, long, required = true, num_args = 1.., value_name = "FILE")]
    input: Vec<String>,
    #[arg(short, long, required = true, value_name = "DIR")]
    output: String,
    #[arg(long, conflicts_with = "cartesian_meters")]
    input_epsg: Option<u16>,
    #[arg(long, conflicts_with = "input_epsg")]
    cartesian_meters: bool,
    #[arg(long, default_value_t = 4979)]
    output_epsg: u16,
    #[arg(long, default_value_t = 15)]
    min: u8,
    #[arg(long, default_value_t = 18)]
    max: u8,
    #[arg(long, default_value_t = 4 * 1024)]
    max_memory_mb: usize,
    #[arg(long, value_name = "N")]
    threads: Option<usize>,
    #[arg(long)]
    quantize: bool,
    #[arg(long)]
    gzip_compress: bool,
    #[arg(long)]
    meshopt: bool,
    #[arg(long)]
    disable_decimation: bool,
}

impl From<Cli> for ConvertOptions {
    fn from(cli: Cli) -> Self {
        let input_reference = if let Some(code) = cli.input_epsg {
            InputReference::Epsg { code }
        } else {
            InputReference::CartesianMeters
        };
        Self {
            input: cli.input,
            output: cli.output,
            input_reference,
            output_epsg: cli.output_epsg,
            min: cli.min,
            max: cli.max,
            max_memory_mb: cli.max_memory_mb,
            threads: cli.threads,
            quantize: cli.quantize,
            gzip_compress: cli.gzip_compress,
            meshopt: cli.meshopt,
            disable_decimation: cli.disable_decimation,
        }
    }
}

fn main() -> std::io::Result<()> {
    Builder::new()
        .format(|buf, record| {
            writeln!(
                buf,
                "{} [{}] - {}",
                Local::now().format("%Y-%m-%d %H:%M:%S"),
                record.level(),
                record.args()
            )
        })
        .filter(None, LevelFilter::Info)
        .init();

    convert(Cli::parse().into())
}
