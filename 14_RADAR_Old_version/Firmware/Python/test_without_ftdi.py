# %% Imports
import time
import logging
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import hilbert

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# Constants for angle estimation
MATRIX_SIZE = 83  # 83x83 matrix
BUFFER_SIZE = MATRIX_SIZE * MATRIX_SIZE  # 6889 elements
AZIMUTH_RANGE = (-41.8, 41.8)  # Azimuth range
ELEVATION_RANGE = (41.8, -41.8)  # Elevation range
REFRESH_RATE = 0.5  # Refresh rate in seconds (adjust as needed)
CFAR_GUARD_CELLS = 2  # Number of guard cells around CUT
CFAR_TRAINING_CELLS = 5  # Number of training cells around CUT
CFAR_THRESHOLD_FACTOR = 1.5  # CFAR scaling factor (reduced for better sensitivity)

# Radar parameters (user-configurable)
sample_rate = 60e6  # 60 MHz
chirp_BW = 30e6  # 30 MHz
num_chirps = 459
NUM_SAMPLES = 14
NUM_CHIRPS = 459
num_samples = 14
ramp_time_s = 0.25e-6  # Ramp time in seconds
frame_length_ms = 0.5e-3  # Frame length in milliseconds
output_freq = 10.5e9
signal_freq = 32e6
max_range = 300  # Maximum range in meters
max_speed = 50  # Maximum speed in m/s
c = 3e8  # Speed of light
wavelength = c / output_freq
slope = chirp_BW / ramp_time_s
PRI = frame_length_ms / 1e3  # Pulse repetition interval
PRF = 1 / PRI  # Pulse repetition frequency
N_frame = int(PRI * float(sample_rate))
freq = np.linspace(-sample_rate / 2, sample_rate / 2, N_frame)
dist = (freq - signal_freq) * c / (2 * slope)
R_res = c / (2 * chirp_BW)
v_res = wavelength / (2 * num_chirps * PRI)
max_doppler_freq = PRF / 2
max_doppler_vel = max_doppler_freq * wavelength / 2

# Plotting parameters
MTI_filter = '2pulse'  # choices are none, 2pulse, or 3pulse
min_scale = 4
max_scale = 300

def generate_radar_data():
    """
    Generate synthetic RADAR data with a target and noise.
    """
    # Create an empty array
    data = np.zeros((MATRIX_SIZE, MATRIX_SIZE), dtype=np.uint8)
    
    # Add a synthetic target (e.g., a high-value spike)
    target_row, target_col = MATRIX_SIZE // 2, MATRIX_SIZE // 2  # Place the target in the middle
    data[target_row, target_col] = 255  # Maximum value for uint8
    
    # Add Gaussian noise
    noise = np.random.normal(0, 50, (MATRIX_SIZE, MATRIX_SIZE)).astype(np.uint8)  # Adjust noise level as needed
    data = np.clip(data + noise, 0, 255)  # Ensure values stay within uint8 range
    
    return data.flatten()  # Return as 1D array

def generate_angle_grid():
    """
    Generate azimuth and elevation matrices.
    """
    azimuth_values = np.linspace(AZIMUTH_RANGE[0], AZIMUTH_RANGE[1], MATRIX_SIZE)
    elevation_values = np.linspace(ELEVATION_RANGE[0], ELEVATION_RANGE[1], MATRIX_SIZE)
    azimuth_matrix, elevation_matrix = np.meshgrid(azimuth_values, elevation_values)
    return azimuth_matrix, elevation_matrix

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
    
    # Debug: Print the number of detected targets
    num_targets = np.count_nonzero(detected_targets)
    logging.info(f"CFAR detected {num_targets} targets.")
    
    return detected_targets

def process_radar_data(buffer):
    """
    Convert buffer into an 83x83 matrix, apply CFAR, and find target angles.
    """
    if len(buffer) != BUFFER_SIZE:
        raise ValueError(f"Invalid buffer size! Expected {BUFFER_SIZE}, got {len(buffer)}")
    
    # Reshape the 1D buffer into an 83x83 matrix
    matrix = buffer.reshape((MATRIX_SIZE, MATRIX_SIZE))
    detected_targets = ca_cfar(matrix)
    max_index = np.unravel_index(np.argmax(detected_targets), matrix.shape)
    azimuth_matrix, elevation_matrix = generate_angle_grid()
    max_azimuth = azimuth_matrix[max_index]
    max_elevation = elevation_matrix[max_index]
    return detected_targets, max_index, max_azimuth, max_elevation

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

# %%
# Function to process data
def pulse_canceller(radar_data):
    global num_chirps, num_samples
    rx_chirps = []
    rx_chirps = radar_data
    # create 2 pulse canceller MTI array
    Chirp2P = np.empty([num_chirps, num_samples])*1j
    for chirp in range(num_chirps-1):
        chirpI = rx_chirps[chirp,:]
        chirpI1 = rx_chirps[chirp+1,:]
        chirp_correlation = np.correlate(chirpI, chirpI1, 'valid')
        angle_diff = np.angle(chirp_correlation, deg=False)  # returns radians
        Chirp2P[chirp,:] = (chirpI1 - chirpI * np.exp(-1j*angle_diff[0]))
    # create 3 pulse canceller MTI array
    Chirp3P = np.empty([num_chirps, num_samples])*1j
    for chirp in range(num_chirps-2):
        chirpI = Chirp2P[chirp,:]
        chirpI1 = Chirp2P[chirp+1,:]
        Chirp3P[chirp,:] = chirpI1 - chirpI
    return Chirp2P, Chirp3P

def freq_process(data):
    """
    Process radar data to generate range-Doppler spectrum.
    """
    # Reshape the 1D data into a 2D matrix
    
    rx_chirps_fft = np.fft.fftshift(abs(np.fft.fft2(data)))
    range_doppler_data = np.log10(rx_chirps_fft).T
    # or this is the longer way to do the fft2 function:
    # rx_chirps_fft = np.fft.fft(data)
    # rx_chirps_fft = np.fft.fft(rx_chirps_fft.T).T   
    # rx_chirps_fft = np.fft.fftshift(abs(rx_chirps_fft))
    range_doppler_data = np.log10(rx_chirps_fft).T
    num_good = len(range_doppler_data[:,0])   
    center_delete = 0  # delete ground clutter velocity bins around 0 m/s
    if center_delete != 0:
        for g in range(center_delete):
            end_bin = int(num_chirps/2+center_delete/2)
            range_doppler_data[:,(end_bin-center_delete+g)] = np.zeros(num_good)
    range_delete = 0   # delete the zero range bins (these are Tx to Rx leakage)
    if range_delete != 0:
        for r in range(range_delete):
            start_bin = int(len(range_doppler_data)/2)
            range_doppler_data[start_bin+r, :] = np.zeros(num_chirps)
    range_doppler_data = np.clip(range_doppler_data, min_scale, max_scale)  # clip the data to control the max spectrogram scale
    


    # Debug: Print the range-Doppler data statistics
    logging.info(f"Range-Doppler data:{range_doppler_data}")
    logging.info(f"Range-Doppler data: min={np.min(range_doppler_data)}, max={np.max(range_doppler_data)}")
    
    return range_doppler_data

k = 0
buffer = generate_radar_data()
data = buffer[:6426]
data = data.reshape((NUM_SAMPLES,NUM_CHIRPS))
k=int((k+1) % len(data))

# %% Main loop
def main():
    try:
        global k
        
        # Initialize plot
        plt.ion()
        
        # Create a figure with two subplots: one for target position and one for range-Doppler spectrum
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
        
        # Initialize the range-Doppler plot
        extent = [-max_doppler_vel, max_doppler_vel, dist.min(), dist.max()]
        cmaps = ['inferno', 'plasma']
        cmn = cmaps[0]
        range_doppler_plot = ax2.imshow(np.zeros((NUM_SAMPLES, NUM_CHIRPS)), aspect='auto', extent=extent, origin='lower', cmap=plt.get_cmap(cmn))
        ax2.set_xlim([-max_speed, max_speed])
        ax2.set_ylim([0, max_range])
        ax2.set_yticks(np.arange(0, max_range, 10))
        ax2.set_ylabel('Range [m]')
        ax2.set_title('Range Doppler Spectrum')
        ax2.set_xlabel('Velocity [m/s]')

        while True:
            # Generate synthetic RADAR data
            buffer = generate_radar_data()
            data = buffer[:6426]
            data = data.reshape((NUM_SAMPLES,NUM_CHIRPS))
    
            
            # Process radar data
            detected_targets, max_pos, max_azimuth, max_elevation = process_radar_data(buffer)
            
            # Print detected target angles
            logging.info(f"Detected Target at {max_pos} -> Azimuth: {max_azimuth:.2f}°, Elevation: {max_elevation:.2f}°")

            # Update target position plot
            plot_target_position(ax1, max_azimuth, max_elevation)

            # Process range-Doppler data
            
            range_doppler_data = freq_process(data)

            # Update range-Doppler plot
            if MTI_filter != 'none':
                Chirp2P, Chirp3P = pulse_canceller(data[k])
                if MTI_filter == '3pulse':
                    freq_process_data = freq_process(Chirp3P)
                else:
                    freq_process_data = freq_process(Chirp2P)
            else:
                freq_process_data = freq_process(data[k])
            range_doppler.set_data(freq_process_data)
            plt.show(block=False)
            plt.pause(0.1)
            if step_thru_plots == True:
                val = input()
                if val == '0':
                    k=int((k-1) % len(buffer))
                else:
                    k=int((k+1) % len(buffer))
            else:
                k=int((k+1) % len(buffer))
            
            #range_doppler_plot.set_data(range_doppler_data)
            #range_doppler_plot.set_clim(vmin=np.min(range_doppler_data), vmax=np.max(range_doppler_data))  # Adjust color scale
            #plt.draw()
            
            # Pause to control the refresh rate
            plt.pause(REFRESH_RATE)

    except KeyboardInterrupt:
        logging.info("Process interrupted by user.")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        logging.info("Processing stopped.")

# %% Run the script
if __name__ == "__main__":
    main()
