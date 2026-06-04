import pathlib

p = pathlib.Path("/home/alpha_now/Desktop/progs/tipe_project/src")
print("hey")

def foreach_file(p: pathlib.Path):
    if p.is_file():
        yield p
    else:
        for q in p.iterdir():
            yield from foreach_file(q)

for q in foreach_file(p):
    print(f"== {q.relative_to(p)}\n\n```{q.read_text()}```\n\n")
