use std::error::Error;

use pcd_core::pointcloud::point::PointCloud;

pub mod csv;
pub mod las;
pub mod ply;

pub trait ParserProvider {
    fn get_parser(&self) -> Box<dyn Parser>;
}

pub trait Parser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Extension {
    Las,
    Laz,
    Csv,
    Txt,
    Ply,
}

pub fn get_extension(extension: &str) -> Extension {
    match extension.to_lowercase().as_str() {
        "las" => Extension::Las,
        "laz" => Extension::Laz,
        "csv" => Extension::Csv,
        "txt" => Extension::Txt,
        "ply" => Extension::Ply,
        _ => panic!("Unsupported extension: {}", extension),
    }
}
