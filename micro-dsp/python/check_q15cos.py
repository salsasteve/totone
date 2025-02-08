import numpy as np

def generate_cosine_table():
    """
    Generate a cosine table similar to the one in the Rust code.
    The table will have 256 entries corresponding to angles from 0 to 2π.
    """
    table_size = 256
    cosine_table = np.zeros(table_size, dtype=np.int16)
    
    for i in range(table_size):
        angle = (2 * np.pi * i) / table_size
        cosine_value = np.cos(angle)
        cosine_table[i] = int(cosine_value * 32767)
    
    return cosine_table

def q15_cos(angle, cosine_table):
    """
    Compute the cosine of an angle using the lookup table.
    
    Parameters:
        angle (int): Angle in Q15 format (0 to 32768 corresponds to 0 to 2π).
        cosine_table (np.ndarray): Lookup table with cosine values.
    
    Returns:
        int: Cosine of the angle in Q15 format.
    """
    index = angle >> 8
    return cosine_table[index]

# Generate the cosine table
cosine_table = generate_cosine_table()

# Test the cosine function with a few angles
test_angles = [0, 8192, 16384, 24576, 32768]
expected_results = [32767, 23169, 0, -23169, -32767]

print("Angle (Q15) | Cosine (Q15) | Expected (Q15)")
print("------------------------------------------")
for angle, expected in zip(test_angles, expected_results):
    result = q15_cos(angle, cosine_table)
    print(f"{angle:10d} | {result:11d} | {expected:12d}")