import numpy as np

def generate_cosine_wave_int16(frequency, sample_rate):
    """
    Generate one period of a cosine wave in int16 format.

    Parameters:
        frequency (float): Frequency of the cosine wave in Hz.
        sample_rate (int): Sample rate in samples per second.

    Returns:
        np.ndarray: Array of int16 values representing one period of the cosine wave.
    """
    # Calculate the period of the wave
    period = 1.0 / frequency

    # Generate time values for one period
    t = np.linspace(0, period, int(sample_rate * period), endpoint=False)

    # Generate the cosine wave
    cosine_wave = np.cos(2 * np.pi * frequency * t)

    # Scale the cosine wave to int16 range
    scaled_wave = cosine_wave * 32767

    # Convert to int16
    int16_wave = scaled_wave.astype(np.int16)

    return int16_wave

# Example usage
frequency = 440  # 440 Hz (A4 note)
sample_rate = 44100  # 44.1 kHz

cosine_wave = generate_cosine_wave_int16(frequency, sample_rate)

# Print the entire list of samples
print(cosine_wave.tolist())