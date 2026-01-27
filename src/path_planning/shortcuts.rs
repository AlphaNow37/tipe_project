use crate::workspace::obstacles::ObstaclesEnv;
use crate::workspace::workspace::WorkspaceTopology;
use rand::{rng, Rng};

fn path_length<W: WorkspaceTopology>(workspace: &W, path: &Vec<W::Segment>) -> f64 {
    path.iter().map(|part| workspace.length(*part)).sum::<f64>()
}

pub fn shortcut<W: WorkspaceTopology>(
    workspace: &W,
    mut path: Vec<W::Segment>,
    obstacles: &impl ObstaclesEnv<W>,
    nb_repeat: usize,
) -> Vec<W::Segment> {
    let mut rng = rng();
    'a: for _ in 0..nb_repeat {
        let base_length = path_length(workspace, &path);
        let f1 = rng.random_range(0.0..base_length);
        let f2 = rng.random_range(f1..=base_length);
        let mut new_path = Vec::new();
        let mut curr_length = 0.;
        let mut inter_pt = None;
        for part in path.iter() {
            let length = workspace.length(*part);
            if curr_length > f2 || curr_length + length < f1 {
                new_path.push(*part);
            } else if curr_length < f1 && curr_length + length >= f1 {
                let first_part = workspace.steer_in_disc(*part, f1 - curr_length);
                new_path.push(first_part);
                inter_pt = Some(workspace.segment_end(first_part));
            }
            if curr_length < f2 && curr_length + length >= f2 {
                let inter = inter_pt.unwrap();
                let second_part = workspace.segment_reverse(
                    workspace
                        .steer_in_disc(workspace.segment_reverse(*part), curr_length + length - f2),
                );
                let inter2 = workspace.segment_start(second_part);
                let inter_segment = workspace.segment(inter, inter2);
                if obstacles.collide_segment(inter_segment) {
                    continue 'a;
                }
                new_path.push(inter_segment);
                new_path.push(second_part);
            }
            curr_length += length;
        }
        if path_length(workspace, &new_path) < base_length {
            path = new_path;
        }
    }
    path
}
