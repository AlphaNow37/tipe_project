import matplotlib.pyplot as plt
import json
from math import log

with open("out/perf_benchmark.json") as f:
    datas = json.load(f)

entries = datas["entries"]
params = datas["params"]

fnaive = lambda n: n**3
fopt1 = lambda n: (n**2) * log(n)

naive_asymp = [fnaive(n) * (entries["naive"][-1]) / fnaive(params[-1]) for n in params]
opt1_asymp = [fopt1(n) * (entries["opt1"][-1]) / fopt1(params[-1]) for n in params]

plt.plot(params, naive_asymp, label="~x^3")
plt.plot(params, opt1_asymp, label="~x^2 log n")

for name, times in entries.items():
    plt.plot(params, times, label=name)

plt.legend()
plt.show()
