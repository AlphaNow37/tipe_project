import matplotlib.pyplot as plt
import csv
from math import log

# fname = "cvg_benchmark_straight"
fname = "perf_benchmark"

with open(f"../out/{fname}.csv") as f:
    reader = csv.DictReader(f, delimiter=";")
    fields = reader.fieldnames
    datas = list(reader)

# Configuration
graphed_items = [
    "time_naive_full",
    "time_opt1_full",
    "time_naive_cache",
    "time_opt1_cache",
    "time_naive_gpu_mat",
    "time_naive_gpu_elist",
    "time_polyanya_lib",

    # "rrt",
    # "rrt_shortcut",
    # "rrt_star",
    # "rrt_star_shortcut",
    # "rrt_nobsp",
    # "rrt_star_nobsp"
]

print(datas)

# main_key = "time"
main_key = "map_nb_vertices"
params = [float(entry[main_key]) for entry in datas]

fnaive = (lambda n: n ** 3, "~|S|^3")
fopt1 = (lambda n: (n ** 2) * log(n), "~|S|^2 log |S|")

asymps = {
    "time_naive_full": fnaive,
    "time_opt1_full": fopt1,
    "time_naive_cache": fnaive,
    "time_opt1_cache": fopt1,
    "time_naive_gpu_mat": fnaive,
    "time_naive_gpu_elist": fnaive,
    "time_polyanya_lib": (lambda n: n*n, "square"),
}
colors = {
    "time_naive_full": "red",
    "time_opt1_full": "green",
    "time_naive_cache": "blue",
    "time_opt1_cache": "purple",
    "time_naive_gpu_mat": "turquoise",
    "time_naive_gpu_elist": "magenta",
    "time_polyanya_lib": "red",
    "rrt": "red",
    "rrt_shortcut": "purple",
    "rrt_star": "green",
    "rrt_star_shortcut": "blue",
    "rrt_nobsp": "orange",
    "rrt_star_nobsp": "turquoise",
}
labels = {
    "time_naive_full": "Algo naif",
    "time_opt1_full": "Algo line sweep",
    "time_naive_cache": "Algo naif, lazy",
    "time_opt1_cache": "Algo line sweep, lazy",
    "time_naive_gpu_mat": "Algo naif, GPU (matrice d'adjacence)",
    "time_naive_gpu_elist": "Algo naif, GPU (liste d'adjacence)",
    "time_polyanya_lib": "Polyanya (lib externe)",

    "rrt": "RRT",
    "rrt_shortcut": "RRT avec shortcut",
    "rrt_star": "RRT*",
    "rrt_star_shortcut": "RRT* avec shortcut",
    "rrt_nobsp": "RRT sans BSP",
    "rrt_star_nobsp": "RRT* sans BSP",
}

for key in graphed_items:
    if key not in fields:
        continue
    values = [float(entry[key]) for entry in datas if entry[key] != ""]
    params2 = [t for (i, t) in enumerate(params) if datas[i][key] != ""]
    plt.semilogx(params2, values, label=labels[key], color=colors[key], )
    if key in asymps:
        (f, label) = asymps[key]
        vals_asymp = [f(n) * (values[-1]) / f(params2[-1]) for n in params2]
        plt.semilogx(params2, vals_asymp, label=label, color=colors[key], linestyle="dashed")

plt.legend()
plt.show()
