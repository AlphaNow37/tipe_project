import matplotlib.pyplot as plt
import json
from math import log

with open("out/perf_benchmark.json") as f:
    datas = json.load(f)

entries = datas["entries"]
params = datas["params"]

fnaive = (lambda n: n**3, "~n^3")
fopt1 = (lambda n: (n**2) * log(n), "~n^2 log n")

asymps = {
    "naive": fnaive,
    "opt1": fopt1,
    "naive_full": fnaive,
    "opt1_full": fopt1,
    "naive_cache": fnaive,
    "opt1_cache": fopt1,
}
colors = {
    "naive": "red",
    "opt1": "green",
    "naive_full": "red",
    "opt1_full": "green",
    "naive_cache": "blue",
    "opt1_cache": "purple",
}

for key, (f, label) in asymps.items():
    if key not in entries:
        continue
    times = entries[key]
    plt.plot(params, times, label=key)
    vals_asymp = [f(n) * (times[-1]) / f(params[-1]) for n in params]
    plt.plot(params, vals_asymp, label=label)

plt.legend()
plt.show()
