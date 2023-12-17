from os import getcwd
from os.path import join, dirname
from math import sqrt

MAX_RADIUS = 5
LINES = 321
COLS = 221

expansion_groups = dict()
CWD = join(getcwd(), dirname(__file__))

for l in range(MAX_RADIUS*10+1):
    for c in range(MAX_RADIUS*10+1):
        dist = sqrt(l*l+c*c)*0.1
        if dist <= MAX_RADIUS:
            if not dist in expansion_groups:
                expansion_groups[dist] = []
            expansion_groups[dist].append((l,c))

            if c>0: expansion_groups[dist].append((l,-c))
            if l>0: expansion_groups[dist].append((-l,c))
            if c>0 and l>0: expansion_groups[dist].append((-l,-c))

ascending_dist = sorted(expansion_groups)
groups_no = len(expansion_groups)

#============================================= prepare text to file

no_of_pos = 0
for g in expansion_groups.values():
    no_of_pos += len(g)

file = []
file.append(f"const int expansion_positions_no = {no_of_pos};\n")
file.append(f"const float expansion_pos_dist[{no_of_pos}] = {{")

for dist in ascending_dist:
    for p in expansion_groups[dist]:
        file.extend([f"{dist}",","])
file.pop()

file.append(f"}};\n\nconst int expansion_pos_l[{no_of_pos}] = {{")

for dist in ascending_dist:
    for p in expansion_groups[dist]:
        file.extend([f"{p[0]}",","])
file.pop()

file.append(f"}};\n\nconst int expansion_pos_c[{no_of_pos}] = {{")

for dist in ascending_dist:
    for p in expansion_groups[dist]:
        file.extend([f"{p[1]}",","])
file.pop()
file.append("};\n")

#============================================= write to file

with open(join(CWD, "expansion_groups.h"), 'w') as f:
    f.write(''.join(file))
