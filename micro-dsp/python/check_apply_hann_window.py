import numpy as np

Q15_SCALE = 32768

def generate_hann_window_q15(window_len):
    if not (window_len & (window_len - 1) == 0) or window_len > 1024:
        raise ValueError("Window length must be a power of 2 and <= 1024")
    
    hann_window = 0.5 * (1 - np.cos(2 * np.pi * np.arange(window_len) / window_len))
    hann_window_q15 = (hann_window * Q15_SCALE).astype(np.int16)
    return hann_window_q15

def apply_hann_window_q15(samples, window_len):
    if len(samples) != window_len:
        raise ValueError("Sample length must match window length")
    
    hann_window_q15 = generate_hann_window_q15(window_len)
    samples_q15 = (samples * Q15_SCALE).astype(np.int16)
    windowed_samples_q15 = ((samples_q15.astype(np.int32) * hann_window_q15.astype(np.int32)) // Q15_SCALE).astype(np.int16)
    return windowed_samples_q15

# Example usage
window_len = 16
samples = np.ones(window_len)

hann_window_q15 = generate_hann_window_q15(window_len)
windowed_samples_q15 = apply_hann_window_q15(samples, window_len)

print("Hann Window (Q15):", hann_window_q15)
print("Windowed Samples (Q15):", windowed_samples_q15)