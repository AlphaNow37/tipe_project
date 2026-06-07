import math

import matplotlib.pyplot as plt
# import csv
from math import log
import scipy

"""
Ce fichier sert à afficher les benchmarks de utils/benchmarks.rs
"""

# fname = "cvg_benchmark_straight"
fname = "perf_benchmark_aggregate_2"
# fname = "perf_benchmark_10"

with open(f"../out/{fname}.json") as f:
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
    # "polyanya_lib_astar_only",
    "polyanya_me_astar",
    # "polyanya_me_dijstra",
    # "polyanya_me_dijstra_exhaustive",
    "polyanya_me_astar_only",
    "polyanya_me_astar_nodelaunay",
    # "polyanya_me_astar_only_nodelaunay",
    # "tri_me",
    # "tri_delaunay_me",
    "theta_star",

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
    "theta_star": fastar,
}
colors_linestyle = {
    "naive_full": ("#FF9999", "-"),
    "opt1_full": ("#AA00AA", "-"),
    "naive_cache": ("#AA0000", "-"),
    "opt1_cache": ("#AA00AA", "-"),
    "naive_gpu_matrix": ("#FFFF33", "-"),
    "naive_gpu_elist": ("#AAAA00", "-"),
    "graph_astar_only": ("#FF0000", "--"),
    "polyanya_lib": ("#00FF00", "-"),
    "polyanya_lib_astar_only": ("#00FF00", "--"),
    "polyanya_me_astar": ("#2222AA", "-"),
    "polyanya_me_dijstra": ("#22FFFF", "-"),
    "polyanya_me_dijstra_exhaustive": ("#00AAAA", "-"),
    "polyanya_me_astar_only": ("#2222FF", "--"),
    "polyanya_me_astar_nodelaunay": ("#00FFAA", "-"),
    "polyanya_me_astar_only_nodelaunay": ("#000077", "--"),
    "tri_me": ("#BB77FF", "-"),
    "tri_delaunay_me": ("#BB0088", "-"),
    "theta_star": ("#999900", "--")

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
    "opt1_full": "Algo Lee",
    "naive_cache": "Algo naïf, lazy",
    "opt1_cache": "Algo line sweep, lazy",
    "naive_gpu_matrix": "Algo naïf, GPU",
    "naive_gpu_elist": "Algo naïf, GPU",
    "graph_astar_only": "A* sur G_vis",
    "polyanya_lib": "Polyanya (lib rust)",
    "polyanya_lib_astar_only": "Polyanya (lib rust), query",
    "polyanya_me_astar": "Polyanya",
    "polyanya_me_dijstra": "Polyanya, mode dijkstra",
    "polyanya_me_dijstra_exhaustive": "Polyanya, sans early return",
    "polyanya_me_astar_only": "Polyanya, query",
    "polyanya_me_astar_nodelaunay": "Polyanya, sans delaunay",
    "polyanya_me_astar_only_nodelaunay": "Polyanya, sans delaunay, query",
    "tri_me": "Triangulation line sweep",
    "tri_delaunay_me": "Triangulation + delaunay flips",
    "theta_star": "Theta*",

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
    if d["x"] > 20:
        points_par_item.setdefault(d["name"], []).append(d)

print(points_par_item)

for key in graphed_items:
    if key not in points_par_item:
        print(f"Key {key} not found")
        continue
    values = [entry["y"] for entry in points_par_item[key]]
    params = [entry["x"] for entry in points_par_item[key]]

    imin = len(params)//2
    reg = scipy.stats.linregress([log(p, 10) for p in params[imin:]], [log(v, 10) for v in values[imin:]])
    print(f"{key:<30}: T={10**reg.intercept:.2} n^{reg.slope:.2}")
    cst = 10**reg.intercept
    lg = math.floor(log(cst, 10))
    frac = cst / (10**lg)
    # print(f"\n{labels[key].replace(" ", " \\; ")} \n {int(round(frac))}*10^{{{lg}}} |S|^{{{reg.slope:.2}}}\n")

    (color, linestyle) = colors_linestyle[key]
    plt.loglog(params, values, label=f"{labels[key]}: T={10**reg.intercept:.2}*|S|^{reg.slope:.2}", color=color, linestyle=linestyle)


    # plt.loglog(params, [10**(log(p, 10) * reg.slope + reg.intercept) for p in params], label=labels[key], color=color, linestyle="--")

    # if key in asymps:
    #     (f, label) = asymps[key]
    #     vals_normalized = [binsearch(f, v, 0, 1000000000) for v in values]
    #     reg = scipy.stats.linregress(params, vals_normalized)
    #     vals_asymp = [f(reg.intercept + reg.slope * n) for n in params]
    #     plt.loglog(params, vals_asymp, label=label, color=color, linestyle="--")

plt.legend()
plt.show()
