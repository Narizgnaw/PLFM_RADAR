import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

# RADAR Parameters
Fs = 60e6  # Sampling rate (60 Msps)
Fc = 10.5e9  # Carrier frequency (10.5 GHz)
Bw = 30e6  # LFM Bandwidth (30 MHz)
T_chirp = 10e-6  # Chirp duration (10 µs)
T_standby = 20e-6  # Chirp standby duration (20 µs)
N_chirps = 50  # Number of chirps per ADAR1000 position
c = 3e8  # Speed of light (m/s)
max_range = 3000  # Maximum range (m)
max_speed = 270 / 3.6  # Maximum speed (m/s)

# Derived Parameters
num_samples = int(Fs * T_chirp)  # Samples per chirp
range_res = c / (2 * Bw)  # Range resolution
PRI = T_chirp + T_standby  # Pulse Repetition Interval
fd_max = max_speed / (c / Fc / 2)  # Maximum Doppler frequency shift

# Generate LFM Chirp
T = np.linspace(0, T_chirp, num_samples, endpoint=False)
ref_chirp = signal.chirp(T, f0=0, f1=Bw, t1=T_chirp, method='linear')
ref_chirp *= np.hamming(num_samples)  # Apply window function

def simulate_targets(num_targets=1):
    targets = []
    for _ in range(num_targets):
        target_range = np.random.uniform(500, max_range)
        target_speed = np.random.uniform(-max_speed, max_speed)
        time_delay = 2 * target_range / c
        doppler_shift = 2 * target_speed * Fc / c
        targets.append((time_delay, doppler_shift))
    return targets

def generate_received_signal(targets):
    rx_signal = np.zeros(num_samples, dtype=complex)
    for time_delay, doppler_shift in targets:
        delayed_chirp = np.roll(ref_chirp, int(time_delay * Fs))
        rx_signal += delayed_chirp * np.exp(1j * 2 * np.pi * doppler_shift * T)
    rx_signal += 0.1 * (np.random.randn(len(rx_signal)) + 1j * np.random.randn(len(rx_signal)))
    return rx_signal

def cfar_ca1D(signal, guard_cells, training_cells, threshold_factor):
    num_cells = len(signal)
    cfar_output = np.zeros(num_cells, dtype=complex)
    
    for i in range(training_cells + guard_cells, num_cells - training_cells - guard_cells):
        noise_level = np.mean(np.abs(signal[i - training_cells - guard_cells:i - guard_cells]))
        threshold = noise_level * threshold_factor
        cfar_output[i] = signal[i] if np.abs(signal[i]) > threshold else 0
    
    return cfar_output

plt.figure(figsize=(10, 6))
while True:
    targets = simulate_targets(num_targets=3)
    rx_signal = generate_received_signal(targets)
    compressed_signal = signal.fftconvolve(rx_signal, ref_chirp[::-1].conj(), mode='same')
    cfar_output = cfar_ca1D(compressed_signal, guard_cells=3, training_cells=10, threshold_factor=3)
    
    # MTI Processing
    mti_filtered = np.diff(compressed_signal)
    fft_doppler = np.fft.fftshift(np.fft.fft(mti_filtered, N_chirps))
    speed_axis = np.linspace(-max_speed, max_speed, N_chirps)
    range_axis = np.linspace(0, max_range, num_samples)
    
    # Apply CFAR to Doppler Spectrum
    cfar_doppler_output = cfar_ca1D(np.abs(fft_doppler), guard_cells=3, training_cells=10, threshold_factor=3)
    
    # Enhanced Range-Doppler Map
    zero_padded_signal = np.pad(compressed_signal, (0, num_samples), mode='constant')
    range_doppler_map = np.abs(np.fft.fftshift(np.fft.fft2(zero_padded_signal.reshape(N_chirps, -1), axes=(0, 1)), axes=0))
    range_doppler_map = 20 * np.log10(range_doppler_map + 1e-6)
    
    # Plot results
    plt.clf()
    plt.imshow(range_doppler_map, aspect='auto', extent=[0, max_range, -max_speed, max_speed], cmap='viridis')
    plt.colorbar(label='Magnitude (dB)')
    plt.xlabel("Range (m)")
    plt.ylabel("Speed (m/s)")
    plt.title("Real-time Range-Speed Detection")
    plt.pause(0.1)
