use std::{error::Error, path::PathBuf};

use pcd_core::pointcloud::point::{EpsgCode, PointCloud};

use crate::reader::{PointReader, ply::PlyPointReader};

use super::{Parser, ParserProvider};

pub struct PlyParserProvider {
    pub filenames: Vec<PathBuf>,
    pub epsg: EpsgCode,
}

impl ParserProvider for PlyParserProvider {
    fn get_parser(&self) -> Box<dyn Parser> {
        Box::new(PlyParser {
            filenames: self.filenames.clone(),
            epsg: self.epsg,
        })
    }
}

pub struct PlyParser {
    pub filenames: Vec<PathBuf>,
    pub epsg: EpsgCode,
}

impl Parser for PlyParser {
    fn parse(&self) -> Result<PointCloud, Box<dyn Error>> {
        let mut reader = PlyPointReader::new(self.filenames.clone())?;
        let mut points = Vec::new();

        while let Some(point) = reader.next_point()? {
            points.push(point);
        }

        let point_cloud = PointCloud::new(points, self.epsg);
        Ok(point_cloud)
    }
}
