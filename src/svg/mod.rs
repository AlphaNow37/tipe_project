/// This directory focuses on writing on svg files to visualize things
use std::{io::Write, path::Path};

use crate::geometry::VecN;
use object::{Style, SvgObject};

use crate::utils::numbers::NotNanF64;

pub mod curves;
pub mod graph;
pub mod grid;
pub mod object;
pub mod rtree;

#[derive(Default)]
pub struct SvgGroup {
    objects: Vec<(Box<dyn SvgObject>, f64, Style)>,
    background: String,
    pub no_margin: bool,
}

impl SvgGroup {
    pub fn push(&mut self, object: impl SvgObject + 'static, height: f64, style: Style) {
        self.objects.push((Box::new(object), height, style));
    }
    pub fn set_background(&mut self, color: String) {
        self.background = color;
    }
    pub fn write(&mut self, writer: &mut impl Write) -> std::io::Result<()> {
        self.objects.sort_by_key(|(_, h, _)| NotNanF64::new(*h));
        let area = self
            .objects
            .iter()
            .map(|obj| obj.0.collide_box())
            .reduce(|a, b| a.join(b))
            .unwrap_or_default();
        let VecN([w, h]) = area.size();
        let (mx, my) = if !self.no_margin {
            (w * 0.1, h * 0.1)
        } else {
            (0., 0.)
        };
        writeln!(
            writer,
            r#"<svg width="{}" height="{}" viewBox="{},{},{},{}" xmlns="http://www.w3.org/2000/svg" style="background: {}">"#,
            w * 20.,
            h * 20.,
            area.start[0] - mx,
            area.start[1] - my,
            w + 2. * mx,
            h + 2. * my,
            &self.background,
        )?;
        for obj in &self.objects {
            obj.0.write(writer, &obj.2)?;
        }
        writeln!(writer, "</svg>")?;
        Ok(())
    }
    pub fn write_to_file(&mut self, path: &Path) {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(path)
            .unwrap();
        self.write(&mut file).unwrap();
    }
}
