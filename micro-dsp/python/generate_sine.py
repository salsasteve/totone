import numpy as np

def sin_2byte_signed(num_points, sampling_rate, freq):
    duration = (num_points - 1) / sampling_rate
    t = np.linspace(0, duration, num_points, endpoint=True)
    sin_float = np.sin(2 * np.pi * freq * t)
    sin_transformed = sin_float * 32767
    return np.asarray(sin_transformed, dtype=np.int16)

def normalize_2byte_signed_to_float(arr):
    return arr / 32767

num_points = 1024  # Duration of the sine wave in seconds
sampling_rate = 44100  # Sampling rate in Hz
freq = 440  # Frequency of the sine wave in Hz

sine_wave = sin_2byte_signed(num_points, sampling_rate, freq)
normalized_sine_wave = normalize_2byte_signed_to_float(sine_wave)

## print a list in rust list format 
print("let sine_wave_i16 = ["+ ", ".join(map(str, sine_wave)) + "];")
print("let normalized_sine_wave = ["+ ", ".join(map(str, normalized_sine_wave)) + "];")

# Calculate hanning window
hanning_window = np.hanning(num_points)
# Apply hanning window to the sine wave
windowed_sine_wave = normalized_sine_wave * hanning_window

print("let windowed_sine_wave = ["+ ", ".join(map(str, windowed_sine_wave)) + "];")

# # Calculate the FFT
fft_data = np.fft.fft(windowed_sine_wave)
fft_freq = np.fft.fftfreq(num_points, 1/sampling_rate)

# # Get the positive frequency bins
positive_freq_bins = fft_freq[:num_points//2]
positive_fft_data = np.abs(fft_data[:num_points//2])

# Get the magnitude of the FFT data

# # Print the frequency bins and corresponding FFT values
# for i in range(len(positive_freq_bins)):
#     print(f"Frequency: {positive_freq_bins[i]:.2f} Hz, FFT Value: {positive_fft_data[i]:.2f}, Complex Value: {fft_data[i]}")

print("let fft_data = ["+ ", ".join(map(str, positive_fft_data)) + "];")
print("let freq_bins = ["+ ", ".join(map(str, positive_freq_bins)) + "];")