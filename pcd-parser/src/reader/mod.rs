pub mod csv;
pub mod las;
pub mod ply;

use pcd_core::pointcloud::point::Point;
use std::io;

pub trait PointReader {
    fn next_point(&mut self) -> io::Result<Option<Point>>;
}
