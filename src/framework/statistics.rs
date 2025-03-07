pub struct StatWatcher {
    runs: Vec<f64>,
}
impl StatWatcher {
    pub fn push(&mut self, val: f64) {
        self.runs.push(val)
    }
    pub fn from_experiment(n: usize, mut exp: impl FnMut() -> f64) -> Self {
        (0..n).map(|_| exp()).collect()
    }
    pub fn from_experiment_threaded(n: usize, exp: impl Fn() -> f64 + Sync) -> Self {
        const THREAD_COUNT: usize = 16;
        let (sender, receiver) = std::sync::mpsc::channel();
        std::thread::scope(|s| {
            let mut handles = Vec::new();
            for _ in 0..THREAD_COUNT {
                handles.push(s.spawn(|| {
                    for _ in 0..n / THREAD_COUNT {
                        sender.send(exp()).unwrap();
                    }
                }));
            }
            for _ in 0..(n % THREAD_COUNT) {
                sender.send(exp()).unwrap();
            }
        });
        drop(sender);
        receiver.into_iter().collect()
    }
    pub fn show_cli_seq(&self) {
        print!("{}", self.runs[0]);
        for v in &self.runs[1..] {
            print!(" | {v}")
        }
        println!();
    }
    pub fn show_cli_stats(&self) {
        let n = self.runs.len();
        let sum: f64 = self.runs.iter().sum();
        let mut r = self.runs.clone();
        r.sort_floats();
        let median = r[n / 2];
        let avg = sum / (n as f64);
        println!("LENGTH: {n}");
        println!("AVG: {avg:.02}");
        println!("MEDIAN: {median:.02}");
    }
}
impl FromIterator<f64> for StatWatcher {
    fn from_iter<T: IntoIterator<Item = f64>>(iter: T) -> Self {
        Self {
            runs: iter.into_iter().collect(),
        }
    }
}
