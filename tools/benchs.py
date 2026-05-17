import matplotlib.pyplot as plt
# import csv
import json
from math import log
import scipy

# fname = "cvg_benchmark_straight"
fname = "perf_benchmark"

with open(f"out/{fname}.json") as f:
    datas = eval(f.read())

# Configuration
graphed_items = [
    "naive_full",
    "opt1_full",
    # "naive_cache",
    # "opt1_cache",
    # "naive_gpu_matrix",
    "naive_gpu_elist",
    "graph_astar_only",
    "polyanya_lib",
    "polyanya_lib_astar_only",
    "polyanya_me_astar",
    "polyanya_me_dijstra",
    "polyanya_me_dijstra_exhaustive",
    "polyanya_me_astar_only",
    "polyanya_me_astar_nodelaunay",
    "polyanya_me_astar_only_nodelaunay",
    "tri_me",
    "tri_delaunay_me",

    # "rrt",
    # "rrt_shortcut",
    # "rrt_star",
    # "rrt_star_shortcut",
    # "rrt_nobsp",
    # "rrt_star_nobsp"
]

print(datas)

# main_key = "time"
# main_key = "map_nb_vertices"

# params = [float(entry[main_key]) for entry in datas]

fnaive = (lambda n: n ** 3, "~|S|^3")
fopt1 = (lambda n: (n ** 2) * log(n), "~|S|^2 log |S|")
# fpolyanya = (lambda n: n, "~ |S|")
fpolyanya = (lambda n: n*n*n, "~ |S|^2")
fastar = (lambda n: n * log(n), "~ |S| log |S|")
ftri = (lambda n: n * log(n), "~ |S| log |S|")

asymps = {
    "naive_full": fnaive,
    "opt1_full": fopt1,
    "naive_cache": fnaive,
    "opt1_cache": fopt1,
    "naive_gpu_matrix": fnaive,
    "naive_gpu_elist": fnaive,
    "graph_astar_only": fastar,
    "polyanya_lib": fpolyanya,
    "polyanya_lib_astar_only": fpolyanya,
    "polyanya_me_astar": fpolyanya,
    "polyanya_me_dijstra": fpolyanya,
    "polyanya_me_dijstra_exhaustive": fpolyanya,
    "polyanya_me_astar_only": fpolyanya,
    "polyanya_me_astar_nodelaunay": fpolyanya,
    "polyanya_me_astar_only_nodelaunay": fpolyanya,
    "tri_me": ftri,
    "tri_delaunay_me": fopt1,
}
colors_linestyle = {
    "naive_full": ("#FF7777", "-"),
    "opt1_full": ("#FF44FF", "-"),
    "naive_cache": ("#AA0000", "-"),
    "opt1_cache": ("#AA00AA", "-"),
    "naive_gpu_matrix": ("#FFFF33", "-"),
    "naive_gpu_elist": ("#AAAA00", "-"),
    "graph_astar_only": ("#FF0000", ":"),
    "polyanya_lib": ("#00FF00", "-"),
    "polyanya_lib_astar_only": ("#00FF00", ":"),
    "polyanya_me_astar": ("#2222FF", "-"),
    "polyanya_me_dijstra": ("#22FFFF", "-"),
    "polyanya_me_dijstra_exhaustive": ("#00AAAA", "-"),
    "polyanya_me_astar_only": ("#2222FF", ":"),
    "polyanya_me_astar_nodelaunay": ("#000077", "-"),
    "polyanya_me_astar_only_nodelaunay": ("#000077", ":"),
    "tri_me": ("#BB77FF", "-"),
    "tri_delaunay_me": ("#BB0088", "-"),

    # "time_naive_full": "red",
    # "time_opt1_full": "green",
    # "time_naive_cache": "blue",
    # "time_opt1_cache": "purple",
    # "time_naive_gpu_mat": "turquoise",
    # "time_naive_gpu_elist": "magenta",
    # "time_polyanya_lib": "red",
    # "rrt": "red",
    # "rrt_shortcut": "purple",
    # "rrt_star": "green",
    # "rrt_star_shortcut": "blue",
    # "rrt_nobsp": "orange",
    # "rrt_star_nobsp": "turquoise",
}
labels = {
    "naive_full": "Algo naïf",
    "opt1_full": "Algo line sweep",
    "naive_cache": "Algo naïf, lazy",
    "opt1_cache": "Algo line sweep, lazy",
    "naive_gpu_matrix": "Algo naïf, GPU (matrice d'adjacence)",
    "naive_gpu_elist": "Algo naïf, GPU (liste d'adjacence)",
    "graph_astar_only": "A* sur G_vis uniquement",
    "polyanya_lib": "Polyanya (lib externe rust)",
    "polyanya_lib_astar_only": "Polyanya (lib externe rust), query uniquement",
    "polyanya_me_astar": "Polyanya, mode a*",
    "polyanya_me_dijstra": "Polyanya, mode dijkstra",
    "polyanya_me_dijstra_exhaustive": "Polyanya, sans early return",
    "polyanya_me_astar_only": "Polyanya, query uniquement",
    "polyanya_me_astar_nodelaunay": "Polyanya, sans delaunay",
    "polyanya_me_astar_only_nodelaunay": "Polyanya, sans delaunay, query uniquement",
    "tri_me": "Triangulation line sweep",
    "tri_delaunay_me": "Triangulation + delaunay flips",

    "rrt": "RRT",
    "rrt_shortcut": "RRT avec shortcut",
    "rrt_star": "RRT*",
    "rrt_star_shortcut": "RRT* avec shortcut",
    "rrt_nobsp": "RRT sans BSP",
    "rrt_star_nobsp": "RRT* sans BSP",
}


def binsearch(f, y, xmin, xmax):
    """Find x such that f(x)~=y. f must be continuous and growing"""
    while xmax-xmin > 0.000001:
        mid = (xmin+xmax)/2
        if f(mid) < y:
            xmin = mid
        else:
            xmax = mid
    return (xmin+xmax)/2


points_par_item = {}
for d in datas:
    points_par_item.setdefault(d["name"], []).append(d)

for key in graphed_items:
    if key not in points_par_item:
        continue
    values = [entry["y"] for entry in points_par_item[key]]
    params = [entry["x"] for entry in points_par_item[key]]
    (color, linestyle) = colors_linestyle[key]
    plt.loglog(params, values, label=labels[key], color=color, linestyle=linestyle)
    # if key in asymps:
    #     (f, label) = asymps[key]
    #     vals_normalized = [binsearch(f, v, 0, 1000000000) for v in values]
    #     reg = scipy.stats.linregress(params, vals_normalized)
    #     vals_asymp = [f(reg.intercept + reg.slope * n) for n in params]
    #     plt.loglog(params, vals_asymp, label=label, color=color, linestyle="--")

plt.legend()
plt.show()
