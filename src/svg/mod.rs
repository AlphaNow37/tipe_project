use std::{io::Write, path::Path};

use object::SvgObject;

use crate::datastructures::traits::NotNanF64;

pub mod object;

#[derive(Default)]
pub struct SvgGroup {
    objects: Vec<(Box<dyn SvgObject>, f64)>,
}

impl SvgGroup {
    pub fn push(&mut self, object: impl SvgObject + 'static, height: f64) {
        self.objects.push((Box::new(object), height));
    }
    pub fn write(&mut self, writer: &mut impl Write) -> std::io::Result<()> {
        self.objects.sort_by_key(|(_, h)| NotNanF64::new(*h));
        let area = self
            .objects
            .iter()
            .map(|obj| obj.0.collide_box())
            .reduce(|a, b| a.join(b))
            .unwrap_or_default();
        writeln!(
            writer,
            r#"<svg width="{}" height="{}" viewBox="{},{},{},{}" xmlns="http://www.w3.org/2000/svg">"#,
            area.size()[0] + 20.,
            area.size()[1] + 20.,
            area.start[0] - 10.,
            area.start[1] - 10.,
            area.size()[0] + 20.,
            area.size()[1] + 20.,
        )?;
        for obj in &self.objects {
            obj.0.write(writer)?;
        }
        writeln!(writer, "</svg>")?;
        Ok(())
    }
    pub fn write_to_file(&mut self, path: &Path) {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .open(path)
            .unwrap();
        self.write(&mut file).unwrap();
    }
}
