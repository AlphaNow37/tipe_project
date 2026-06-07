from PIL.Image import open
import pathlib

"""
Ce fichier sert à convertir une image noir/blanc en grille d'accessibilité qui peut être ouverte coté rust
"""

print("opening")

input_folder = pathlib.Path(__file__).parent.parent / "inputs"
input = open(input_folder / "circuit_tetra_1.png")

print(input)

result = []
start_pos = None
end_pos = None
for y in range(input.height):
    print(f"line: {y}/{input.height}")
    for x in range(input.width):
        pixel = input.getpixel((x, y))
        if pixel[0] > 128 or pixel[1] > 128:
            value = 1
        else:
            value = 0
        if pixel[0] > 128 and pixel[2] < 128:
            start_pos = (x, y)
        if pixel[1] > 128 and pixel[2] < 128:
            end_pos = (x, y)
        result.append(str(value))
    result.append("\n")

result.insert(0, f"{start_pos[0]}\n{start_pos[1]}\n{end_pos[0]}\n{end_pos[1]}\n")

print("saving")

(input_folder / "circuit_tetra_1_arr.txt").write_text("".join(result))
