use std::{
    error::Error,
    fs::File,
    io::{self, BufRead, BufReader, Read, Seek, SeekFrom},
    path::PathBuf,
};

use pcd_core::pointcloud::point::{Color, Point, PointAttributes};

use super::PointReader;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlyFormat {
    Ascii,
    BinaryLittleEndian,
    BinaryBigEndian,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PropertyType {
    Char,   // i8
    UChar,  // u8
    Short,  // i16
    UShort, // u16
    Int,    // i32
    UInt,   // u32
    Float,  // f32
    Double, // f64
}

impl PropertyType {
    pub fn size_in_bytes(&self) -> usize {
        match self {
            PropertyType::Char | PropertyType::UChar => 1,
            PropertyType::Short | PropertyType::UShort => 2,
            PropertyType::Int | PropertyType::UInt | PropertyType::Float => 4,
            PropertyType::Double => 8,
        }
    }

    pub fn parse(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "char" | "int8" => Some(PropertyType::Char),
            "uchar" | "uint8" => Some(PropertyType::UChar),
            "short" | "int16" => Some(PropertyType::Short),
            "ushort" | "uint16" => Some(PropertyType::UShort),
            "int" | "int32" => Some(PropertyType::Int),
            "uint" | "uint32" => Some(PropertyType::UInt),
            "float" | "float32" => Some(PropertyType::Float),
            "double" | "float64" => Some(PropertyType::Double),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PropertyDef {
    pub name: String,
    pub data_type: PropertyType,
    pub offset: usize, // byte offset within a binary vertex record
}

#[derive(Debug, Clone)]
pub struct PlyHeader {
    pub format: PlyFormat,
    pub vertex_count: usize,
    pub properties: Vec<PropertyDef>,
    pub vertex_stride: usize,
    pub header_byte_size: u64,

    // Property indices
    pub x_prop: Option<usize>,
    pub y_prop: Option<usize>,
    pub z_prop: Option<usize>,
    pub r_prop: Option<usize>,
    pub g_prop: Option<usize>,
    pub b_prop: Option<usize>,
    pub intensity_prop: Option<usize>,
}

impl PlyHeader {
    pub fn parse<R: Read + Seek>(reader: &mut R) -> Result<Self, Box<dyn Error + Send + Sync>> {
        let mut buf_reader = BufReader::new(reader);
        let mut line = String::new();

        let mut byte_pos = 0u64;

        // Check magic number
        let bytes_read = buf_reader.read_line(&mut line)?;
        if bytes_read == 0 || !line.trim().eq_ignore_ascii_case("ply") {
            return Err("Not a valid PLY file (missing 'ply' magic header)".into());
        }
        byte_pos += bytes_read as u64;

        let mut format = None;
        let mut in_vertex_element = false;
        let mut vertex_count = 0usize;
        let mut properties = Vec::new();
        let mut current_offset = 0usize;

        loop {
            line.clear();
            let bytes = buf_reader.read_line(&mut line)?;
            if bytes == 0 {
                return Err("Unexpected EOF while reading PLY header".into());
            }
            byte_pos += bytes as u64;

            let trimmed = line.trim();
            if trimmed.is_empty() || trimmed.starts_with("comment") || trimmed.starts_with("obj_info") {
                continue;
            }

            let parts: Vec<&str> = trimmed.split_whitespace().collect();
            if parts.is_empty() {
                continue;
            }

            match parts[0].to_lowercase().as_str() {
                "format" => {
                    if parts.len() < 2 {
                        return Err("Malformed 'format' line in PLY header".into());
                    }
                    format = match parts[1].to_lowercase().as_str() {
                        "ascii" => Some(PlyFormat::Ascii),
                        "binary_little_endian" => Some(PlyFormat::BinaryLittleEndian),
                        "binary_big_endian" => Some(PlyFormat::BinaryBigEndian),
                        other => return Err(format!("Unsupported PLY format '{}'", other).into()),
                    };
                }
                "element" => {
                    if parts.len() < 3 {
                        return Err("Malformed 'element' line in PLY header".into());
                    }
                    if parts[1].eq_ignore_ascii_case("vertex") {
                        in_vertex_element = true;
                        vertex_count = parts[2].parse::<usize>()?;
                    } else {
                        in_vertex_element = false;
                    }
                }
                "property" => {
                    if in_vertex_element {
                        if parts.len() < 3 {
                            return Err("Malformed 'property' line in PLY header".into());
                        }
                        if parts[1].eq_ignore_ascii_case("list") {
                            return Err("List properties in vertex element are not supported".into());
                        }
                        let prop_type = PropertyType::parse(parts[1])
                            .ok_or_else(|| format!("Unknown PLY property type '{}'", parts[1]))?;
                        let prop_name = parts[2].to_lowercase();
                        let size = prop_type.size_in_bytes();
                        properties.push(PropertyDef {
                            name: prop_name,
                            data_type: prop_type,
                            offset: current_offset,
                        });
                        current_offset += size;
                    }
                }
                "end_header" => {
                    break;
                }
                _ => {}
            }
        }

        let format = format.ok_or("Missing 'format' specification in PLY header")?;

        let mut x_prop = None;
        let mut y_prop = None;
        let mut z_prop = None;
        let mut r_prop = None;
        let mut g_prop = None;
        let mut b_prop = None;
        let mut intensity_prop = None;

        for (idx, prop) in properties.iter().enumerate() {
            match prop.name.as_str() {
                "x" => x_prop = Some(idx),
                "y" => y_prop = Some(idx),
                "z" => z_prop = Some(idx),
                "r" | "red" | "diffuse_red" => r_prop = Some(idx),
                "g" | "green" | "diffuse_green" => g_prop = Some(idx),
                "b" | "blue" | "diffuse_blue" => b_prop = Some(idx),
                "intensity" | "scalar_intensity" => intensity_prop = Some(idx),
                _ => {}
            }
        }

        if x_prop.is_none() || y_prop.is_none() || z_prop.is_none() {
            return Err("PLY file vertex element missing required 'x', 'y', or 'z' properties".into());
        }

        Ok(PlyHeader {
            format,
            vertex_count,
            properties,
            vertex_stride: current_offset,
            header_byte_size: byte_pos,
            x_prop,
            y_prop,
            z_prop,
            r_prop,
            g_prop,
            b_prop,
            intensity_prop,
        })
    }
}

enum ActiveReader {
    Ascii {
        reader: BufReader<File>,
    },
    Binary {
        reader: BufReader<File>,
        buffer: Vec<u8>,
    },
}

pub struct PlyPointReader {
    pub files: Vec<PathBuf>,
    pub current_file_index: usize,
    current_header: Option<PlyHeader>,
    current_reader: Option<ActiveReader>,
    points_read_in_file: usize,
}

impl PlyPointReader {
    pub fn new(files: Vec<PathBuf>) -> io::Result<Self> {
        let mut reader = Self {
            files,
            current_file_index: 0,
            current_header: None,
            current_reader: None,
            points_read_in_file: 0,
        };
        reader.open_next_file()?;
        Ok(reader)
    }

    pub fn estimate_processing_size(path: &PathBuf) -> u64 {
        if let Ok(mut file) = File::open(path) {
            if let Ok(header) = PlyHeader::parse(&mut file) {
                let bytes_per_point = if header.vertex_stride > 0 {
                    header.vertex_stride as u64
                } else {
                    32
                };
                return header.vertex_count as u64 * bytes_per_point;
            }
        }
        path.metadata().map(|m| m.len()).unwrap_or(0)
    }

    fn open_next_file(&mut self) -> io::Result<()> {
        if self.current_file_index < self.files.len() {
            let path = &self.files[self.current_file_index];
            self.current_file_index += 1;

            let mut file = File::open(path)?;
            let header = PlyHeader::parse(&mut file).map_err(io::Error::other)?;

            file.seek(SeekFrom::Start(header.header_byte_size))?;

            let active_reader = match header.format {
                PlyFormat::Ascii => ActiveReader::Ascii {
                    reader: BufReader::new(file),
                },
                PlyFormat::BinaryLittleEndian | PlyFormat::BinaryBigEndian => {
                    ActiveReader::Binary {
                        reader: BufReader::new(file),
                        buffer: vec![0u8; header.vertex_stride],
                    }
                }
            };

            self.current_header = Some(header);
            self.current_reader = Some(active_reader);
            self.points_read_in_file = 0;
            Ok(())
        } else {
            self.current_header = None;
            self.current_reader = None;
            self.points_read_in_file = 0;
            Ok(())
        }
    }

    fn read_scalar_f64(slice: &[u8], prop_type: PropertyType, is_le: bool) -> f64 {
        match prop_type {
            PropertyType::Char => slice[0] as i8 as f64,
            PropertyType::UChar => slice[0] as f64,
            PropertyType::Short => {
                let v = if is_le {
                    i16::from_le_bytes([slice[0], slice[1]])
                } else {
                    i16::from_be_bytes([slice[0], slice[1]])
                };
                v as f64
            }
            PropertyType::UShort => {
                let v = if is_le {
                    u16::from_le_bytes([slice[0], slice[1]])
                } else {
                    u16::from_be_bytes([slice[0], slice[1]])
                };
                v as f64
            }
            PropertyType::Int => {
                let v = if is_le {
                    i32::from_le_bytes(slice[0..4].try_into().unwrap())
                } else {
                    i32::from_be_bytes(slice[0..4].try_into().unwrap())
                };
                v as f64
            }
            PropertyType::UInt => {
                let v = if is_le {
                    u32::from_le_bytes(slice[0..4].try_into().unwrap())
                } else {
                    u32::from_be_bytes(slice[0..4].try_into().unwrap())
                };
                v as f64
            }
            PropertyType::Float => {
                let v = if is_le {
                    f32::from_le_bytes(slice[0..4].try_into().unwrap())
                } else {
                    f32::from_be_bytes(slice[0..4].try_into().unwrap())
                };
                v as f64
            }
            PropertyType::Double => {
                if is_le {
                    f64::from_le_bytes(slice[0..8].try_into().unwrap())
                } else {
                    f64::from_be_bytes(slice[0..8].try_into().unwrap())
                }
            }
        }
    }

    fn extract_color_component(slice: &[u8], prop_type: PropertyType, is_le: bool) -> u16 {
        match prop_type {
            PropertyType::Char => (slice[0] as u16).saturating_mul(257),
            PropertyType::UChar => (slice[0] as u16).saturating_mul(257),
            PropertyType::Short => {
                let v = if is_le {
                    i16::from_le_bytes([slice[0], slice[1]])
                } else {
                    i16::from_be_bytes([slice[0], slice[1]])
                };
                v.max(0) as u16
            }
            PropertyType::UShort => {
                if is_le {
                    u16::from_le_bytes([slice[0], slice[1]])
                } else {
                    u16::from_be_bytes([slice[0], slice[1]])
                }
            }
            PropertyType::Int | PropertyType::UInt => {
                let v = Self::read_scalar_f64(slice, prop_type, is_le);
                if v <= 255.0 {
                    (v.max(0.0) as u16).saturating_mul(257)
                } else {
                    v.clamp(0.0, 65535.0) as u16
                }
            }
            PropertyType::Float | PropertyType::Double => {
                let v = Self::read_scalar_f64(slice, prop_type, is_le);
                if v <= 1.0 && v >= 0.0 {
                    (v * 65535.0).round().clamp(0.0, 65535.0) as u16
                } else if v <= 255.0 {
                    (v.max(0.0) as u16).saturating_mul(257)
                } else {
                    v.clamp(0.0, 65535.0) as u16
                }
            }
        }
    }

    fn parse_ascii_color_component(val_str: &str, prop_type: PropertyType) -> u16 {
        let v: f64 = val_str.parse().unwrap_or(255.0);
        match prop_type {
            PropertyType::Char | PropertyType::UChar => (v.clamp(0.0, 255.0) as u16).saturating_mul(257),
            PropertyType::Short | PropertyType::UShort => v.clamp(0.0, 65535.0) as u16,
            PropertyType::Int | PropertyType::UInt => {
                if v <= 255.0 {
                    (v.clamp(0.0, 255.0) as u16).saturating_mul(257)
                } else {
                    v.clamp(0.0, 65535.0) as u16
                }
            }
            PropertyType::Float | PropertyType::Double => {
                if v <= 1.0 && v >= 0.0 {
                    (v * 65535.0).round().clamp(0.0, 65535.0) as u16
                } else if v <= 255.0 {
                    (v.clamp(0.0, 255.0) as u16).saturating_mul(257)
                } else {
                    v.clamp(0.0, 65535.0) as u16
                }
            }
        }
    }
}

impl PointReader for PlyPointReader {
    fn next_point(&mut self) -> io::Result<Option<Point>> {
        loop {
            let header = match self.current_header.as_ref() {
                Some(h) => h,
                None => {
                    self.open_next_file()?;
                    match self.current_header.as_ref() {
                        Some(h) => h,
                        None => return Ok(None),
                    }
                }
            };

            if self.points_read_in_file >= header.vertex_count {
                self.open_next_file()?;
                if self.current_header.is_none() {
                    return Ok(None);
                }
                continue;
            }

            let header = self.current_header.as_ref().unwrap();
            let active_reader = self.current_reader.as_mut().unwrap();

            match active_reader {
                ActiveReader::Ascii { reader } => {
                    let mut line = String::new();
                    loop {
                        line.clear();
                        let bytes = reader.read_line(&mut line)?;
                        if bytes == 0 {
                            self.open_next_file()?;
                            if self.current_header.is_none() {
                                return Ok(None);
                            }
                            break;
                        }
                        let trimmed = line.trim();
                        if trimmed.is_empty() {
                            continue;
                        }

                        let tokens: Vec<&str> = trimmed.split_whitespace().collect();
                        if tokens.len() < header.properties.len() {
                            return Err(io::Error::other(format!(
                                "Corrupted PLY ascii vertex: expected {} tokens, found {}",
                                header.properties.len(),
                                tokens.len()
                            )));
                        }

                        let x_idx = header.x_prop.unwrap();
                        let y_idx = header.y_prop.unwrap();
                        let z_idx = header.z_prop.unwrap();

                        let x: f64 = tokens[x_idx]
                            .parse()
                            .map_err(|e| io::Error::other(format!("Failed to parse 'x': {}", e)))?;
                        let y: f64 = tokens[y_idx]
                            .parse()
                            .map_err(|e| io::Error::other(format!("Failed to parse 'y': {}", e)))?;
                        let z: f64 = tokens[z_idx]
                            .parse()
                            .map_err(|e| io::Error::other(format!("Failed to parse 'z': {}", e)))?;

                        let r = header
                            .r_prop
                            .map(|idx| {
                                Self::parse_ascii_color_component(
                                    tokens[idx],
                                    header.properties[idx].data_type,
                                )
                            })
                            .unwrap_or(65535);

                        let g = header
                            .g_prop
                            .map(|idx| {
                                Self::parse_ascii_color_component(
                                    tokens[idx],
                                    header.properties[idx].data_type,
                                )
                            })
                            .unwrap_or(65535);

                        let b = header
                            .b_prop
                            .map(|idx| {
                                Self::parse_ascii_color_component(
                                    tokens[idx],
                                    header.properties[idx].data_type,
                                )
                            })
                            .unwrap_or(65535);

                        let intensity = header.intensity_prop.and_then(|idx| {
                            tokens[idx].parse::<f64>().ok().map(|v| v.clamp(0.0, 65535.0) as u16)
                        });

                        self.points_read_in_file += 1;

                        return Ok(Some(Point {
                            x,
                            y,
                            z,
                            color: Color { r, g, b },
                            attributes: PointAttributes {
                                intensity,
                                return_number: None,
                                classification: None,
                                scanner_channel: None,
                                scan_angle: None,
                                user_data: None,
                                point_source_id: None,
                                gps_time: None,
                            },
                        }));
                    }
                }
                ActiveReader::Binary { reader, buffer } => {
                    match reader.read_exact(buffer) {
                        Ok(()) => {
                            let is_le = header.format == PlyFormat::BinaryLittleEndian;

                            let x_idx = header.x_prop.unwrap();
                            let y_idx = header.y_prop.unwrap();
                            let z_idx = header.z_prop.unwrap();

                            let x_def = &header.properties[x_idx];
                            let y_def = &header.properties[y_idx];
                            let z_def = &header.properties[z_idx];

                            let x = Self::read_scalar_f64(
                                &buffer[x_def.offset..x_def.offset + x_def.data_type.size_in_bytes()],
                                x_def.data_type,
                                is_le,
                            );
                            let y = Self::read_scalar_f64(
                                &buffer[y_def.offset..y_def.offset + y_def.data_type.size_in_bytes()],
                                y_def.data_type,
                                is_le,
                            );
                            let z = Self::read_scalar_f64(
                                &buffer[z_def.offset..z_def.offset + z_def.data_type.size_in_bytes()],
                                z_def.data_type,
                                is_le,
                            );

                            let r = header
                                .r_prop
                                .map(|idx| {
                                    let def = &header.properties[idx];
                                    Self::extract_color_component(
                                        &buffer[def.offset..def.offset + def.data_type.size_in_bytes()],
                                        def.data_type,
                                        is_le,
                                    )
                                })
                                .unwrap_or(65535);

                            let g = header
                                .g_prop
                                .map(|idx| {
                                    let def = &header.properties[idx];
                                    Self::extract_color_component(
                                        &buffer[def.offset..def.offset + def.data_type.size_in_bytes()],
                                        def.data_type,
                                        is_le,
                                    )
                                })
                                .unwrap_or(65535);

                            let b = header
                                .b_prop
                                .map(|idx| {
                                    let def = &header.properties[idx];
                                    Self::extract_color_component(
                                        &buffer[def.offset..def.offset + def.data_type.size_in_bytes()],
                                        def.data_type,
                                        is_le,
                                    )
                                })
                                .unwrap_or(65535);

                            let intensity = header.intensity_prop.map(|idx| {
                                let def = &header.properties[idx];
                                let v = Self::read_scalar_f64(
                                    &buffer[def.offset..def.offset + def.data_type.size_in_bytes()],
                                    def.data_type,
                                    is_le,
                                );
                                v.clamp(0.0, 65535.0) as u16
                            });

                            self.points_read_in_file += 1;

                            return Ok(Some(Point {
                                x,
                                y,
                                z,
                                color: Color { r, g, b },
                                attributes: PointAttributes {
                                    intensity,
                                    return_number: None,
                                    classification: None,
                                    scanner_channel: None,
                                    scan_angle: None,
                                    user_data: None,
                                    point_source_id: None,
                                    gps_time: None,
                                },
                            }));
                        }
                        Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => {
                            self.open_next_file()?;
                            if self.current_header.is_none() {
                                return Ok(None);
                            }
                        }
                        Err(e) => return Err(e),
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    #[test]
    fn test_parse_ascii_ply() {
        let ply_content = b"ply
format ascii 1.0
comment generated test
element vertex 2
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
1.0 2.0 3.0 255 0 128
4.5 5.5 6.5 0 255 64
";
        let mut tmp = NamedTempFile::new().unwrap();
        tmp.write_all(ply_content).unwrap();
        tmp.flush().unwrap();

        let mut reader = PlyPointReader::new(vec![tmp.path().to_path_buf()]).unwrap();
        let p1 = reader.next_point().unwrap().expect("p1 should exist");
        assert_eq!(p1.x, 1.0);
        assert_eq!(p1.y, 2.0);
        assert_eq!(p1.z, 3.0);
        assert_eq!(p1.color.r, 255 * 257);
        assert_eq!(p1.color.g, 0);
        assert_eq!(p1.color.b, 128 * 257);

        let p2 = reader.next_point().unwrap().expect("p2 should exist");
        assert_eq!(p2.x, 4.5);
        assert_eq!(p2.y, 5.5);
        assert_eq!(p2.z, 6.5);
        assert_eq!(p2.color.r, 0);
        assert_eq!(p2.color.g, 255 * 257);
        assert_eq!(p2.color.b, 64 * 257);

        assert!(reader.next_point().unwrap().is_none());
    }

    #[test]
    fn test_parse_binary_little_endian_ply() {
        let header = b"ply\nformat binary_little_endian 1.0\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
        let mut data = Vec::new();
        data.extend_from_slice(header);

        // Point 1: (10.0f32, 20.0f32, 30.0f32), (255u8, 128u8, 0u8)
        data.extend_from_slice(&10.0f32.to_le_bytes());
        data.extend_from_slice(&20.0f32.to_le_bytes());
        data.extend_from_slice(&30.0f32.to_le_bytes());
        data.push(255);
        data.push(128);
        data.push(0);

        // Point 2: (100.0f32, 200.0f32, 300.0f32), (0u8, 64u8, 255u8)
        data.extend_from_slice(&100.0f32.to_le_bytes());
        data.extend_from_slice(&200.0f32.to_le_bytes());
        data.extend_from_slice(&300.0f32.to_le_bytes());
        data.push(0);
        data.push(64);
        data.push(255);

        let mut tmp = NamedTempFile::new().unwrap();
        tmp.write_all(&data).unwrap();
        tmp.flush().unwrap();

        let mut reader = PlyPointReader::new(vec![tmp.path().to_path_buf()]).unwrap();
        let p1 = reader.next_point().unwrap().expect("p1 exists");
        assert_eq!(p1.x, 10.0);
        assert_eq!(p1.y, 20.0);
        assert_eq!(p1.z, 30.0);
        assert_eq!(p1.color.r, 65535);
        assert_eq!(p1.color.g, 128 * 257);
        assert_eq!(p1.color.b, 0);

        let p2 = reader.next_point().unwrap().expect("p2 exists");
        assert_eq!(p2.x, 100.0);
        assert_eq!(p2.y, 200.0);
        assert_eq!(p2.z, 300.0);
        assert_eq!(p2.color.r, 0);
        assert_eq!(p2.color.g, 64 * 257);
        assert_eq!(p2.color.b, 65535);

        assert!(reader.next_point().unwrap().is_none());
    }

    #[test]
    fn test_parse_binary_big_endian_ply() {
        let header = b"ply\nformat binary_big_endian 1.0\nelement vertex 1\nproperty double x\nproperty double y\nproperty double z\nproperty ushort r\nproperty ushort g\nproperty ushort b\nend_header\n";
        let mut data = Vec::new();
        data.extend_from_slice(header);

        data.extend_from_slice(&123.456f64.to_be_bytes());
        data.extend_from_slice(&(-789.012f64).to_be_bytes());
        data.extend_from_slice(&345.678f64.to_be_bytes());
        data.extend_from_slice(&50000u16.to_be_bytes());
        data.extend_from_slice(&30000u16.to_be_bytes());
        data.extend_from_slice(&10000u16.to_be_bytes());

        let mut tmp = NamedTempFile::new().unwrap();
        tmp.write_all(&data).unwrap();
        tmp.flush().unwrap();

        let mut reader = PlyPointReader::new(vec![tmp.path().to_path_buf()]).unwrap();
        let p = reader.next_point().unwrap().expect("point exists");
        assert!((p.x - 123.456).abs() < 1e-6);
        assert!((p.y - (-789.012)).abs() < 1e-6);
        assert!((p.z - 345.678).abs() < 1e-6);
        assert_eq!(p.color.r, 50000);
        assert_eq!(p.color.g, 30000);
        assert_eq!(p.color.b, 10000);

        assert!(reader.next_point().unwrap().is_none());
    }

    #[test]
    fn test_ply_default_color() {
        let ply_content = b"ply
format ascii 1.0
element vertex 1
property float x
property float y
property float z
end_header
10.0 20.0 30.0
";
        let mut tmp = NamedTempFile::new().unwrap();
        tmp.write_all(ply_content).unwrap();
        tmp.flush().unwrap();

        let mut reader = PlyPointReader::new(vec![tmp.path().to_path_buf()]).unwrap();
        let p = reader.next_point().unwrap().unwrap();
        assert_eq!(p.x, 10.0);
        assert_eq!(p.y, 20.0);
        assert_eq!(p.z, 30.0);
        assert_eq!(p.color.r, 65535);
        assert_eq!(p.color.g, 65535);
        assert_eq!(p.color.b, 65535);
    }
}
