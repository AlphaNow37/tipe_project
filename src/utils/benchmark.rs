use std::{collections::HashMap, io::Write, path::Path, time::Instant};

pub trait ToJson {
    fn to_json(&self, writer: &mut impl Write) -> std::io::Result<()>;
}
impl ToJson for f64 {
    fn to_json(&self, writer: &mut impl Write) -> std::io::Result<()> {
        write!(writer, "{:?}", self)
    }
}
impl ToJson for usize {
    fn to_json(&self, writer: &mut impl Write) -> std::io::Result<()> {
        write!(writer, "{:?}", self)
    }
}
impl<T: ToJson> ToJson for Vec<T> {
    fn to_json(&self, writer: &mut impl Write) -> std::io::Result<()> {
        write!(writer, "[")?;
        if self.len() > 0 {
            for i in 0..(self.len() - 1) {
                self[i].to_json(writer)?;
                write!(writer, ",")?;
            }
            self.last().unwrap().to_json(writer)?
        }
        write!(writer, "]")
    }
}
impl<T: ToJson> ToJson for HashMap<String, T> {
    fn to_json(&self, writer: &mut impl Write) -> std::io::Result<()> {
        write!(writer, "{{")?;
        if self.len() > 0 {
            let mut keys = self.keys();
            let k = keys.next().unwrap();
            write!(writer, "\"{}\":", k)?;
            self[k].to_json(writer)?;
            for k in keys {
                write!(writer, ",\"{}\":", k)?;
                self[k].to_json(writer)?;
            }
        }
        write!(writer, "}}")
    }
}

struct Entry<P, V> {
    values: Vec<V>,
    func: Box<dyn Fn(&P) -> V>,
}
impl<P, V: ToJson> ToJson for Entry<P, V> {
    fn to_json(&self, writer: &mut impl Write) -> std::io::Result<()> {
        self.values.to_json(writer)
    }
}

pub struct Benchmark<P, V, U> {
    entries: HashMap<String, Entry<P, V>>,
    params: Vec<U>,
    p_translater: Box<dyn Fn(&P) -> U>,
}

impl<P, V: ToJson, U: ToJson> Benchmark<P, V, U> {
    pub fn new(p_translater: impl Fn(&P) -> U + 'static) -> Self {
        Self {
            entries: HashMap::new(),
            params: Vec::new(),
            p_translater: Box::new(p_translater),
        }
    }
    pub fn add_func(&mut self, f: impl Fn(&P) -> V + 'static, name: impl Into<String>) {
        assert!(self.params.len() == 0);
        self.entries.insert(
            name.into(),
            Entry {
                values: Vec::new(),
                func: Box::new(f),
            },
        );
    }
    pub fn add_param(&mut self, param: &P) {
        for e in &mut self.entries.values_mut() {
            e.values.push((e.func)(&param));
        }
        self.params.push((self.p_translater)(&param));
    }
    pub fn write_to_file(&self, path: &Path) -> std::io::Result<()> {
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .truncate(true)
            .create(true)
            .open(path)
            .unwrap();
        write!(file, "{{\"entries\": ")?;
        self.entries.to_json(&mut file)?;
        write!(file, ", \"params\": ")?;
        self.params.to_json(&mut file)?;
        write!(file, "}}")
    }
}

pub fn time_bench<P>(f: impl Fn(&P) + 'static) -> impl Fn(&P) -> f64 {
    move |p| {
        let start = Instant::now();
        f(p);
        start.elapsed().as_secs_f64()
    }
}
