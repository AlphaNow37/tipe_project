use std::{fmt::Display, io::Write};

use crate::geometry::{shapes::Cube, VecN};

pub trait SvgObject {
    fn write(&self, writer: &mut dyn Write) -> std::io::Result<()>;
    fn collide_box(&self) -> Cube<2>;
}

fn s(obj: &impl Display) -> String {
    format!("\"{}\"", obj.to_string())
}

pub struct Polygon {
    pub points: Vec<VecN<2, f64>>,
    pub color: String,
}
impl SvgObject for Polygon {
    fn write(&self, writer: &mut dyn Write) -> std::io::Result<()> {
        write!(writer, "<polygon fill={} points=\"", s(&self.color))?;
        for pt in &self.points {
            write!(writer, "{},{} ", pt[0], pt[1])?;
        }
        writeln!(writer, "\"/>")?;
        Ok(())
    }
    fn collide_box(&self) -> Cube<2> {
        self.points
            .iter()
            .map(|pt| Cube::from_point(*pt))
            .reduce(Cube::join)
            .unwrap_or_default()
    }
}

pub struct Line {
    pub start: VecN<2, f64>,
    pub end: VecN<2, f64>,
    pub color: String,
}
impl SvgObject for Line {
    fn write(&self, writer: &mut dyn Write) -> std::io::Result<()> {
        write!(
            writer,
            "<line fill={} x1={} y1={} x2={} y2={}",
            s(&self.color),
            s(&self.start[0]),
            s(&self.start[1]),
            s(&self.end[0]),
            s(&self.end[1]),
        )?;
        writeln!(writer, "/>")?;
        Ok(())
    }
    fn collide_box(&self) -> Cube<2> {
        Cube::join(Cube::from_point(self.start), Cube::from_point(self.end))
    }
}
