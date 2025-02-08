import numpy as np

def inverse_q15_cos(cos_value):
    """
    Compute the angle in Q15 format (0 to 32768 corresponds to 0 to 2π) for a given cosine value.

    Parameters:
        cos_value (float): Cosine value in the range [-1, 1].

    Returns:
        int: Angle in Q15 format (0 to 32768 corresponds to 0 to 2π).
    """
    # Compute the arccos of the cosine value
    angle_rad = np.arccos(cos_value)

    # Convert radians to Q15 format
    angle_q15 = int((angle_rad / (2 * np.pi)) * 32768)

    return angle_q15

# Example usage
cos_values = [1.0, 0.5, 0.0, -0.7071, -1.0]
angles_q15 = [inverse_q15_cos(cos_value) for cos_value in cos_values]

print("Cosine Value | Angle (Q15)")
print("--------------------------")
for cos_value, angle in zip(cos_values, angles_q15):
    print(f"{cos_value:12.4f} | {angle:10d}")