use std::{fmt::Display, io::Write};

use crate::geometry::shapes::{Cube, Polygon, Segment};

#[derive(Default, Clone, Debug)]
pub struct Style {
    stroke: Option<(String, f64)>,
    fill: Option<String>,
}
impl Style {
    pub fn stroke(col: String, width: f64) -> Self {
        Self {
            stroke: Some((col, width)),
            ..Self::default()
        }
    }
    pub fn fill(col: String) -> Self {
        Self {
            fill: Some(col),
            ..Self::default()
        }
    }
}
impl Display for Style {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(col) = &self.fill {
            write!(f, "fill=\"{}\" ", col)?;
        }
        if let Some((col, width)) = &self.stroke {
            write!(f, "stroke=\"{}\" stroke-width=\"{}\" ", col, width)?;
        }
        Ok(())
    }
}

pub trait SvgObject {
    fn write(&self, writer: &mut dyn Write, style: &Style) -> std::io::Result<()>;
    fn collide_box(&self) -> Cube<2>;
}

fn s(obj: &impl Display) -> String {
    format!("\"{}\"", obj.to_string())
}

impl SvgObject for Polygon {
    fn write(&self, writer: &mut dyn Write, style: &Style) -> std::io::Result<()> {
        write!(writer, "<polygon {} points=\"", style)?;
        for pt in &self.0 {
            write!(writer, "{},{} ", pt[0], pt[1])?;
        }
        writeln!(writer, "\"/>")?;
        Ok(())
    }
    fn collide_box(&self) -> Cube<2> {
        self.0
            .iter()
            .map(|pt| Cube::from_point(*pt))
            .reduce(Cube::join)
            .unwrap_or_default()
    }
}

impl SvgObject for Segment<2> {
    fn write(&self, writer: &mut dyn Write, style: &Style) -> std::io::Result<()> {
        write!(
            writer,
            "<line {} x1={} y1={} x2={} y2={}",
            style,
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

impl SvgObject for Cube<2> {
    fn write(&self, writer: &mut dyn Write, style: &Style) -> std::io::Result<()> {
        write!(
            writer,
            "<rectangle {} x={} y={} width={} height={}",
            style,
            s(&self.start[0]),
            s(&self.start[1]),
            s(&self.size()[0]),
            s(&self.size()[1]),
        )?;
        writeln!(writer, "/>")?;
        Ok(())
    }
    fn collide_box(&self) -> Cube<2> {
        *self
    }
}
