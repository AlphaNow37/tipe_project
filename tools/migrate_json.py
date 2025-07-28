import json


with open(
    "/home/alpha_now/Desktop/progs/tipe_project/out/perf_benchmark copy.json"
) as inp:
    data = json.load(inp)

params = data["params"]
opt1 = data["entries"]["opt1"]
naive = data["entries"]["naive"]

with open(
    "/home/alpha_now/Desktop/progs/tipe_project/out/nice_perf_benchmark.csv", "w"
) as out:
    out.write("nb_map_vertices;time_naive_full;time_opt1_full\n")
    for p, n, o in zip(params, naive, opt1):
        out.write(f"{p};{n};{o};\n")
