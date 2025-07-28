import matplotlib.pyplot as plt
import csv
from math import log

with open("out/nice_perf_benchmark.csv") as f:
    reader = csv.DictReader(f, delimiter=";")
    fields = reader.fieldnames
    datas = list(reader)


# Configuration
graphed_items = [
    "time_naive_full",
    "time_opt1_full",
    "time_naive_cache",
    "time_opt1_cache",
]
params = [int(entry["nb_map_vertices"]) for entry in datas]


fnaive = (lambda n: n**3, "~n^3")
fopt1 = (lambda n: (n**2) * log(n), "~n^2 log n")
asymps = {
    "time_naive_full": fnaive,
    "time_opt1_full": fopt1,
    "time_naive_cache": fnaive,
    "time_opt1_cache": fopt1,
}
colors = {
    "time_naive_full": "red",
    "time_opt1_full": "green",
    "time_naive_cache": "blue",
    "time_opt1_cache": "purple",
}
labels = {
    "time_naive_full": "Algo naif",
    "time_opt1_full": "Algo rayon tournant",
    "time_naive_cache": "Algo naif, lazy",
    "time_opt1_cache": "ALgo rayon tournant, lazy",
}

for key in graphed_items:
    if key not in fields:
        continue
    times = [float(entry[key]) for entry in datas]
    plt.plot(params, times, label=labels[key], color=colors[key])
    if key in asymps:
        (f, label) = asymps[key]
        vals_asymp = [f(n) * (times[-1]) / f(params[-1]) for n in params]
        plt.plot(params, vals_asymp, label=label, color=colors[key], linestyle="dashed")

plt.legend()
plt.show()
