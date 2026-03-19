# %% Imports
import time
import logging
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from pyftdi.ftdi import Ftdi
from pyftdi.usbtools import UsbTools

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# %% FT2232HQ Configuration
VID = 0x0403  # FTDI Vendor ID
PID = 0x6010  # FT2232HQ Product ID
INTERFACE = 0  # Interface A (0) or B (1)

# Buffer to store received data
buffer = bytearray()
buf_size = 6889

all_data =[]

# Constants for angle estimation
MATRIX_SIZE = 83  # 83x83 matrix
BUFFER_SIZE = MATRIX_SIZE * MATRIX_SIZE  # 6889 elements
AZIMUTH_RANGE = (-41.8, 41.8)  # Azimuth range
ELEVATION_RANGE = (41.8, -41.8)  # Elevation range
REFRESH_RATE = 0.5  # Refresh rate in seconds (adjust as needed)
CFAR_GUARD_CELLS = 2  # Number of guard cells around CUT
CFAR_TRAINING_CELLS = 5  # Number of training cells around CUT
CFAR_THRESHOLD_FACTOR = 3  # CFAR scaling factor

# Radar parameters
sample_rate = 60e6
signal_freq = 30e6
output_freq = 10.5e9
num_chirps = 256
chirp_BW = 25e6
ramp_time_s = 0.25e-6
frame_length_ms = 0.5e-3  # each chirp is spaced this far apart
c = 3e8  # Speed of light
wavelength = c / output_freq
slope = chirp_BW / ramp_time_s
PRI = frame_length_ms / 1e3
PRF = 1 / PRI
N_frame = int(PRI * float(sample_rate))
freq = np.linspace(-sample_rate / 2, sample_rate / 2, N_frame)
dist = (freq - signal_freq) * c / (2 * slope)
R_res = c / (2 * chirp_BW)
v_res = wavelength / (2 * num_chirps * PRI)
max_doppler_freq = PRF / 2
max_doppler_vel = max_doppler_freq * wavelength / 2

# Plotting parameters
MTI_filter = '2pulse'  # choices are none, 2pulse, or 3pulse
max_range = 300
min_scale = 4
max_scale = 300


# %% Initialize FT2232HQ
def initialize_ft2232hq(ftdi):
    """
    Initialize the FT2232HQ for synchronous FIFO mode.
    """
    try:
        # Open the FTDI device in FT245 FIFO mode (Interface A)
        ftdi.open(vendor=VID, product=PID, interface=INTERFACE)
        
        # Reset the FT2232HQ
        ftdi.set_bitmode(0x00, Ftdi.BitMode.RESET)
        time.sleep(0.1)

        # Configure the FT2232HQ for synchronous FIFO mode
        ftdi.set_bitmode(0x00, Ftdi.BitMode.SYNCFF)
        time.sleep(0.1)
        # Set USB transfer latency timer (reduce to improve real-time data)
        ftdi.write_data_set_chunksize(BUFFER_SIZE)
        ftdi.read_data_set_chunksize(BUFFER_SIZE)
        ftdi.set_latency_timer(2)  # 2ms latency timer
        ftdi.purge_buffers()



        # Enable synchronous FIFO mode
        ftdi.write_data(bytes([0x00]))  # Dummy write to activate FIFO mode

        logging.info("FT2232HQ initialized in synchronous FIFO mode.")
        return ftdi
    except Exception as e:
        logging.error(f"Failed to initialize FT2232HQ: {e}")
        raise

# %% Read data from FTDI
def read_data_from_ftdi(ftdi, num_bytes=BUFFER_SIZE):
    """
    Read data from FT2232H FIFO and store it in a buffer.
    """
    buffer = bytearray()
    try:
        while len(buffer) < num_bytes:
            data = ftdi.read_data(num_bytes - len(buffer))
            if data:
                buffer.extend(data)
            time.sleep(0.001)  # Small delay to avoid high CPU usage
    except Exception as e:
        logging.error(f"Read error: {e}")
        raise

    logging.info(f"Read {len(buffer)} bytes from FT2232H")
    return buffer

# %% Generate angle grid
def generate_angle_grid():
    """
    Generate azimuth and elevation matrices.
    """
    azimuth_values = np.linspace(AZIMUTH_RANGE[0], AZIMUTH_RANGE[1], MATRIX_SIZE)
    elevation_values = np.linspace(ELEVATION_RANGE[0], ELEVATION_RANGE[1], MATRIX_SIZE)
    azimuth_matrix, elevation_matrix = np.meshgrid(azimuth_values, elevation_values)
    return azimuth_matrix, elevation_matrix

# %% CFAR Detection
def ca_cfar(matrix):
    """
    Perform CFAR detection on the matrix using vectorized operations.
    """
    detected_targets = np.zeros_like(matrix)
    for i in range(CFAR_TRAINING_CELLS + CFAR_GUARD_CELLS, MATRIX_SIZE - CFAR_TRAINING_CELLS - CFAR_GUARD_CELLS):
        for j in range(CFAR_TRAINING_CELLS + CFAR_GUARD_CELLS, MATRIX_SIZE - CFAR_TRAINING_CELLS - CFAR_GUARD_CELLS):
            # Define CFAR window
            window = matrix[i - CFAR_TRAINING_CELLS - CFAR_GUARD_CELLS : i + CFAR_TRAINING_CELLS + CFAR_GUARD_CELLS + 1,
                            j - CFAR_TRAINING_CELLS - CFAR_GUARD_CELLS : j + CFAR_TRAINING_CELLS + CFAR_GUARD_CELLS + 1]
            guard_area = matrix[i - CFAR_GUARD_CELLS : i + CFAR_GUARD_CELLS + 1,
                                j - CFAR_GUARD_CELLS : j + CFAR_GUARD_CELLS + 1]
            training_cells = np.setdiff1d(window, guard_area)
            noise_level = np.mean(training_cells)
            threshold = CFAR_THRESHOLD_FACTOR * noise_level
            if matrix[i, j] > threshold:
                detected_targets[i, j] = matrix[i, j]
    return detected_targets

# %% Process radar data
def process_radar_data(buffer):
    """
    Convert buffer into an 83x83 matrix, apply CFAR, and find target angles.
    """
    if len(buffer) != BUFFER_SIZE:
        raise ValueError(f"Invalid buffer size! Expected {BUFFER_SIZE}, got {len(buffer)}")
    
    matrix = np.array(buffer).reshape((MATRIX_SIZE, MATRIX_SIZE))
    detected_targets = ca_cfar(matrix)
    max_index = np.unravel_index(np.argmax(detected_targets), matrix.shape)
    azimuth_matrix, elevation_matrix = generate_angle_grid()
    max_azimuth = azimuth_matrix[max_index]
    max_elevation = elevation_matrix[max_index]
    return detected_targets, max_index, max_azimuth, max_elevation

# %% Plot target position
def plot_target_position(ax, azimuth, elevation):
    """
    Plot detected targets with CFAR and refresh display.
    """
    ax.clear()
    ax.set_xlim(AZIMUTH_RANGE)
    ax.set_ylim(ELEVATION_RANGE)
    ax.set_xlabel("Azimuth (°)")
    ax.set_ylabel("Elevation (°)")
    ax.set_title("RADAR Target Detection (CFAR)")
    ax.scatter(azimuth, elevation, color='red', s=100, label="Detected Target")
    ax.legend()
    plt.draw()
    plt.pause(0.01)

# %% Pulse Canceller
def pulse_canceller(radar_data):
    """
    Apply 2-pulse or 3-pulse MTI filtering to radar data.
    """
    global num_chirps, num_samples
    rx_chirps = radar_data
    Chirp2P = np.empty([num_chirps, num_samples], dtype=complex)
    for chirp in range(num_chirps - 1):
        chirpI = rx_chirps[chirp, :]
        chirpI1 = rx_chirps[chirp + 1, :]
        chirp_correlation = np.correlate(chirpI, chirpI1, 'valid')
        angle_diff = np.angle(chirp_correlation, deg=False)  # returns radians
        Chirp2P[chirp, :] = (chirpI1 - chirpI * np.exp(-1j * angle_diff[0]))
    
    Chirp3P = np.empty([num_chirps, num_samples], dtype=complex)
    for chirp in range(num_chirps - 2):
        chirpI = Chirp2P[chirp, :]
        chirpI1 = Chirp2P[chirp + 1, :]
        Chirp3P[chirp, :] = chirpI1 - chirpI
    
    return Chirp2P, Chirp3P

# %% Frequency Processing
def freq_process(data):
    """
    Process radar data to generate range-Doppler spectrum.
    """
    rx_chirps_fft = np.fft.fftshift(abs(np.fft.fft2(data)))
    range_doppler_data = np.log10(rx_chirps_fft).T
    num_good = len(range_doppler_data[:, 0])
    
    # Delete ground clutter velocity bins around 0 m/s
    center_delete = 0
    if center_delete != 0:
        for g in range(center_delete):
            end_bin = int(num_chirps / 2 + center_delete / 2)
            range_doppler_data[:, (end_bin - center_delete + g)] = np.zeros(num_good)
    
    # Delete the zero range bins (Tx to Rx leakage)
    range_delete = 0
    if range_delete != 0:
        for r in range(range_delete):
            start_bin = int(len(range_doppler_data) / 2)
            range_doppler_data[start_bin + r, :] = np.zeros(num_chirps)
    
    # Clip the data to control the max spectrogram scale
    range_doppler_data = np.clip(range_doppler_data, min_scale, max_scale)
    return range_doppler_data

# Plot range doppler data, loop through at the end of the data set
cmn = ''
i = 0
# Initialize FTDI device
ftdi = Ftdi()
initialize_ft2232hq(ftdi)

buffer = read_data_from_ftdi(ftdi, num_bytes=BUFFER_SIZE)
all_data = buffer
raw_data = freq_process(all_data[i])
i=int((i+1) % len(all_data))
range_doppler_fig, ax = plt.subplots(1, figsize=(7,7))
extent = [-max_doppler_vel, max_doppler_vel, dist.min(), dist.max()]
cmaps = ['inferno', 'plasma']
cmn = cmaps[0]
ax.set_xlim([-12, 12])
ax.set_ylim([0, max_range])
ax.set_yticks(np.arange(0, max_range, 10))
ax.set_ylabel('Range [m]')
ax.set_title('Range Doppler Spectrum')
ax.set_xlabel('Velocity [m/s]')
range_doppler = ax.imshow(raw_data, aspect='auto', extent=extent, origin='lower', cmap=matplotlib.colormaps.get_cmap(cmn))

print("CTRL + c to stop the loop")
step_thru_plots = False
if step_thru_plots == True:
    print("Press Enter key to adance to next frame")
    print("Press 0 then Enter to go back one frame")
	
	
# %% Main loop
def main():
    try:
        # Initialize plot
        plt.ion()
        fig, ax = plt.subplots(figsize=(6, 6))

        while True:
            # Read data from FTDI
            buffer = read_data_from_ftdi(ftdi, num_bytes=BUFFER_SIZE)
            all_data = buffer
            detected_targets, max_pos, max_azimuth, max_elevation = process_radar_data(buffer)

            # Print detected target angles
            logging.info(f"Detected Target at {max_pos} -> Azimuth: {max_azimuth:.2f}°, Elevation: {max_elevation:.2f}°")

            # Update plot
            plot_target_position(ax, max_azimuth, max_elevation)
            time.sleep(REFRESH_RATE)
			
            if MTI_filter != 'none':
                Chirp2P, Chirp3P = pulse_canceller(all_data[i])
                if MTI_filter == '3pulse':
                    freq_process_data = freq_process(Chirp3P)
                else:
                    freq_process_data = freq_process(Chirp2P)
            else:
                freq_process_data = freq_process(all_data[i])
            range_doppler.set_data(freq_process_data)
            plt.show(block=False)
            plt.pause(0.1)
            if step_thru_plots == True:
                val = input()
                if val == '0':
                    i=int((i-1) % len(all_data))
                else:
                    i=int((i+1) % len(all_data))
            else:
                i=int((i+1) % len(all_data))

    except KeyboardInterrupt:
        logging.info("Process interrupted by user.")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        ftdi.close()
        logging.info("FTDI device closed.")

# %% Run the script
if __name__ == "__main__":
    main()
