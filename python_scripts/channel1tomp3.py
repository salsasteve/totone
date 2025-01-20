import pandas as pd
import matplotlib.pyplot as plt
import wave, struct
import numpy as np
import argparse
from scipy.signal import butter, lfilter

def plot_waveform(samples, sample_rate):
    time = np.arange(len(samples)) / sample_rate
    plt.figure(figsize=(15, 5))
    plt.plot(time, samples)
    plt.title("I2S Audio Waveform")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.grid(True)
    plt.show()

def normalize_audio(samples):
    max_amplitude = np.max(np.abs(samples))
    if max_amplitude == 0:
        return samples
    return (samples / max_amplitude) * (2**15 - 1)  # Normalize to 16-bit range

def apply_limiter(samples, threshold=0.9):
    max_amplitude = np.max(np.abs(samples))
    if max_amplitude > threshold * (2**15 - 1):
        samples = samples * (threshold * (2**15 - 1) / max_amplitude)
    return samples

def butter_bandpass(lowcut, highcut, fs, order=5):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    return b, a

def bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y

def write_wav(samples, sample_rate, output_file):
    normalized_samples = normalize_audio(samples)
    limited_samples = apply_limiter(normalized_samples)
    
    with wave.open(output_file, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        for s in limited_samples:
            wf.writeframesraw(struct.pack("<h", int(s)))  # Write as 16-bit

def main():
    parser = argparse.ArgumentParser(description='Process I2S data from Saleae export')
    parser.add_argument('input_file', help='Input CSV file from Saleae')
    parser.add_argument('output_file', help='Output WAV file')
    parser.add_argument('--sample-rate', type=int, default=44100, help='Sample rate in Hz')
    parser.add_argument('--lowcut', type=float, default=20.0, help='Low cutoff frequency in Hz')
    parser.add_argument('--highcut', type=float, default=20000.0, help='High cutoff frequency in Hz')
    parser.add_argument('--plot', action='store_true', help='Show waveform plot')
    args = parser.parse_args()

    df = pd.read_csv(args.input_file)
    samples = df[df["Channel"] == 1]["Value"].values  # Extract Channel 1 values
    
    # Apply bandpass filter to allow only human audio frequencies
    filtered_samples = bandpass_filter(samples, args.lowcut, args.highcut, args.sample_rate)
    
    # Normalize samples to 16-bit range
    normalized_samples = normalize_audio(filtered_samples)
    
    # Apply limiter to prevent distortion
    limited_samples = apply_limiter(normalized_samples)
    
    # Write to WAV file
    write_wav(limited_samples, args.sample_rate, args.output_file)
    
    # Optionally plot the waveform
    if args.plot:
        plot_waveform(limited_samples, args.sample_rate)

if __name__ == "__main__":
    main()