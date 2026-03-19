import numpy as np

# Parameters
f0 = 1e6  # Start frequency (Hz)
f1 = 15e6  # End frequency (Hz)
fs = 120e6  # Sampling frequency (Hz)
T = 1e-6  # Time duration (s)
N = int(fs * T)  # Number of samples

# Frequency slope
k = (f1 - f0) / T

# Generate time array
t = np.arange(N) / fs

# Calculate phase
phase = 2 * np.pi * (f0 * t + 0.5 * k * t**2)

# Generate sine wave and convert to 8-bit values
waveform_LUT = np.uint8(128 + 127 * np.sin(phase))

# Print the LUT in Verilog format
print("waveform_LUT[0] = 8'h{:02X};".format(waveform_LUT[0]))
for i in range(1, N):
    print("waveform_LUT[{}] = 8'h{:02X};".format(i, waveform_LUT[i]))
