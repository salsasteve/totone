import math

# Number of entries in the cosine table
TABLE_SIZE = 1024

# Generate the cosine table
cosine_table = []
for i in range(TABLE_SIZE):
    angle = (2 * math.pi * i) / TABLE_SIZE
    cos_value = math.cos(angle)
    q15_value = int(cos_value * 32767)
    cosine_table.append(q15_value)

# Print the cosine table in the format used in the Rust code
print(f"const COSINE_TABLE: [i16; {TABLE_SIZE}] = [")
for i, value in enumerate(cosine_table):
    if i % 8 == 0:
        print("    ", end="")
    print(f"{value:6}, ", end="")
    if (i + 1) % 8 == 0:
        print()
print("];")