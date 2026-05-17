/// A benchmarking tool
/// It creates a json of the form list[{name: string, x: float, y: float}]
use std::{fmt::Display, io::Write, path::PathBuf, time::Instant};

struct DataPoint {
    name: &'static str,
    x: f64,
    y: f64,
}

pub struct Benchmark {
    values: Vec<DataPoint>,
    save_path: PathBuf,
}
impl Benchmark {
    pub fn new(save_path: PathBuf) -> Self {
        Self {
            values: Vec::new(),
            save_path,
        }
    }
    fn try_save(&self) -> Result<(), std::io::Error> {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(&self.save_path)?;

        writeln!(file, "[")?;
        for v in &self.values {
            writeln!(file, r#"    {{"name": "{}", "x": {}, "y": {} }},"#, v.name, v.x, v.y)?;
        }
        writeln!(file, "]")?;
        Ok(())
    }
    fn save(&self) {
        if let Err(e) = self.try_save() {
            eprintln!("Error while saving: {:?}", e);
        }
    }
    pub fn add_datapoint(&mut self, name: &'static str, x: f64, y: f64) {
        self.values.push(DataPoint {name, x, y});
        self.save();
    }
}

pub fn time_bench<T>(f: impl FnOnce() -> T) -> f64 {
    let start = Instant::now();
    f();
    start.elapsed().as_secs_f64()
}
