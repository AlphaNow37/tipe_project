/// A benchmarking tool
/// I export everything to csv and open it in python/excel
use std::{fmt::Display, io::Write, path::PathBuf, time::Instant};

pub struct Benchmark {
    columns: Vec<Vec<String>>,
    nb_rows: usize,
    save_path: PathBuf,
}
impl Benchmark {
    pub fn new(save_path: PathBuf) -> Self {
        Self {
            columns: Vec::new(),
            nb_rows: 0,
            save_path,
        }
    }
    fn try_save(&self) -> Result<(), std::io::Error> {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(&self.save_path)?;
        for i in 0..self.nb_rows {
            for col in &self.columns {
                write!(file, "{};", col[i])?;
            }
            write!(file, "\n")?;
        }
        Ok(())
    }
    fn save(&self) {
        if let Err(e) = self.try_save() {
            eprintln!("Error while saving: {:?}", e);
        }
    }
    pub fn add_column<V: Display>(&mut self, name: impl ToString, values: &[V]) {
        if self.columns.is_empty() {
            self.nb_rows = values.len();
        }
        assert_eq!(values.len(), self.nb_rows);
        let mut column = vec![name.to_string()];
        for v in values {
            column.push(format!("{}", v));
        }
        self.columns.push(column);
        self.save();
    }
    pub fn add_row(&mut self, values: Vec<String>) {
        if self.nb_rows == 0 {
            self.columns = (0..values.len()).map(|_| Vec::new()).collect();
        }
        assert_eq!(values.len(), self.columns.len());
        for (i, v) in values.into_iter().enumerate() {
            self.columns[i].push(v);
        }
        self.nb_rows += 1;
        self.save();
    }
}

pub fn time_bench<P, T>(f: impl Fn(&P) -> T + 'static) -> impl Fn(&P) -> f64 {
    move |p| {
        let start = Instant::now();
        f(p);
        start.elapsed().as_secs_f64()
    }
}
