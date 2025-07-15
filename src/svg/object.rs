/// Implement the protocol for drawing various objects

use std::{fmt::Display, io::Write};
use std::sync::Arc;

use crate::geometry::{
    shapes::{Cube, Polygon, Segment},
    VecN,
};

#[derive(Default, Clone, Debug)]
pub struct Style {
    stroke: Option<(Arc<String>, f64)>,
    fill: Option<Arc<String>>,
}
impl Style {
    pub fn stroke(col: impl Into<String>, width: f64) -> Self {
        Self {
            stroke: Some((Arc::new(col.into()), width)),
            ..Self::default()
        }
    }
    pub fn fill(col: impl Into<String>) -> Self {
        Self {
            fill: Some(Arc::new(col.into())),
            ..Self::default()
        }
    }
    pub fn with_stroke(mut self, col: impl Into<String>, width: f64) -> Self {
        self.stroke = Some((Arc::new(col.into()), width));
        self
    }
    pub fn with_fill(mut self, col: impl Into<String>) -> Self {
        self.fill = Some(Arc::new(col.into()));
        self
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
        write!(writer, "\t<polygon {} points=\"", style)?;
        for pt in self.points() {
            write!(writer, "{},{} ", pt[0], pt[1])?;
        }
        writeln!(writer, "\"/>")?;
        Ok(())
    }
    fn collide_box(&self) -> Cube<2> {
        self.points()
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
            "\t<line {} x1={} y1={} x2={} y2={}",
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
            "\t<rectangle {} x={} y={} width={} height={}",
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

impl SvgObject for Vec<VecN<2, f64>> {
    fn write(&self, writer: &mut dyn Write, style: &Style) -> std::io::Result<()> {
        write!(writer, "\t<polyline {} points=\"", style)?;
        for pt in self {
            write!(writer, "{},{} ", pt[0], pt[1])?;
        }
        writeln!(writer, "\"/>")?;
        Ok(())
    }
    fn collide_box(&self) -> Cube<2> {
        self.iter()
            .map(|pt| Cube::from_point(*pt))
            .reduce(Cube::join)
            .unwrap_or_default()
    }
}
