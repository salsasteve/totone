import numpy as np

def calculate_fft_frequencies(sampling_rate, num_samples):
  """Calculates the frequencies corresponding to FFT bins.

  Args:
    sampling_rate: The sampling rate in Hz.
    num_samples: The number of samples in the FFT (e.g., 1024).

  Returns:
    A NumPy array of frequencies.
  """
  frequency_resolution = sampling_rate / num_samples
  frequencies = np.arange(num_samples) * frequency_resolution
  return frequencies

# Example usage:
sampling_rate = 44100  # Hz
num_samples = 1024
frequencies = calculate_fft_frequencies(sampling_rate, num_samples)

# Print some frequencies
print(f"Bin 0: {frequencies[0]:.2f} Hz")
print(f"Bin 1: {frequencies[1]:.2f} Hz")
print(f"Bin 10: {frequencies[10]:.2f} Hz")
print(f"Bin 512: {frequencies[512]:.2f} Hz")
print(f"Bin 1023: {frequencies[1023]:.2f} Hz")

#A more efficient method using numpy's built in function
fft_freqs = np.fft.fftfreq(num_samples, 1/sampling_rate)
print(f"Bin 0 (np.fft.fftfreq): {fft_freqs[0]:.2f} Hz")
print(f"Bin 1 (np.fft.fftfreq): {fft_freqs[1]:.2f} Hz")
print(f"Bin 10 (np.fft.fftfreq): {fft_freqs[10]:.2f} Hz")
print(f"Bin 512 (np.fft.fftfreq): {fft_freqs[512]:.2f} Hz")
print(f"Bin 1023 (np.fft.fftfreq): {fft_freqs[1023]:.2f} Hz")


#If you only want the positive frequencies, you can use rfftfreq:
rfft_freqs = np.fft.rfftfreq(num_samples, 1/sampling_rate)
print(f"Bin 0 (np.fft.rfftfreq): {rfft_freqs[0]:.2f} Hz")
print(f"Bin 1 (np.fft.rfftfreq): {rfft_freqs[1]:.2f} Hz")
print(f"Bin 10 (np.fft.rfftfreq): {rfft_freqs[10]:.2f} Hz")
print(f"Bin 512 (np.fft.rfftfreq): {rfft_freqs[512]:.2f} Hz")
#Notice there are only 513 bins now.  The last bin is at the Nyquist frequency.p