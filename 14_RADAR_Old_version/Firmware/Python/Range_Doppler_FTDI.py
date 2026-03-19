# %%
# Copyright (C) 2024 Analog Devices, Inc.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#     - Neither the name of Analog Devices, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#     - The use of this software may or may not infringe the patent rights
#       of one or more patent holders.  This license does not release you
#       from the requirement that you obtain separate licenses from these
#       patent holders to use this software.
#     - Use of the software either in source or binary form, must be run
#       on or directly connected to an Analog Devices Inc. component.
#
# THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED.
#
# IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, INTELLECTUAL PROPERTY
# RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# Imports
import time
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from pyftdi.ftdi import Ftdi
from pyftdi.usbtools import UsbTools


# FT2232HQ Configuration

VID = 0x0403  # FTDI Vendor ID
PID = 0x6010  # FT2232HQ Product ID
INTERFACE = 1  # Interface A (1) or B (2)

# Buffer to store received data
buffer = bytearray()
buf_size = 6889

# Constants for angle estimation
MATRIX_SIZE = 83  # 83x83 matrix
BUFFER_SIZE = MATRIX_SIZE * MATRIX_SIZE  # 6889 elements
AZIMUTH_RANGE = (-41.8, 41.8)  # Azimuth range
ELEVATION_RANGE = (41.8, -41.8)  # Elevation range
REFRESH_RATE = 0.5  # Refresh rate in seconds (adjust as needed)
CFAR_GUARD_CELLS = 2  # Number of guard cells around CUT
CFAR_TRAINING_CELLS = 5  # Number of training cells around CUT
CFAR_THRESHOLD_FACTOR = 3  # CFAR scaling factor

plt.close('all')

MTI_filter = '2pulse'  # choices are none, 2pulse, or 3pulse
max_range = 300
min_scale = 4
max_scale = 300


# %%
""" Calculate and print summary of ramp parameters
"""
sample_rate = 60e6
signal_freq = 30e6
output_freq = 10.5e9
num_chirps = 256
chirp_BW = 25e6
ramp_time_s = 0.25e-6
frame_length_ms = 0.5e-6 # each chirp is spaced this far apart
num_samples = len(all_data[0][0])

PRI = frame_length_ms / 1e3
PRF = 1 / PRI

# Split into frames
N_frame = int(PRI * float(sample_rate))

# Obtain range-FFT x-axis
c = 3e8
wavelength = c / output_freq
slope = chirp_BW / ramp_time_s
freq = np.linspace(-sample_rate / 2, sample_rate / 2, N_frame)
dist = (freq - signal_freq) * c / (2 * slope)

# Resolutions
R_res = c / (2 * chirp_BW)
v_res = wavelength / (2 * num_chirps * PRI)

# Doppler spectrum limits
max_doppler_freq = PRF / 2
max_doppler_vel = max_doppler_freq * wavelength / 2

print("sample_rate = ", sample_rate/1e6, "MHz, ramp_time = ", int(ramp_time_s*(1e6)), "us, num_chirps = ", num_chirps, ", PRI = ", frame_length_ms, " ms")

def initialize_ft2232hq(ftdi):
    """
    Initialize the FT2232HQ for synchronous FIFO mode.
    """
    # Open the FTDI device in FT245 FIFO mode (Interface A)
    ftdi.open(VID, PID, INTERFACE)
    # Reset the FT2232HQ
    ftdi.set_bitmode(0x00, Ftdi.BitMode.RESET)
    time.sleep(0.1)

    # Configure the FT2232HQ for synchronous FIFO mode
    ftdi.set_bitmode(0x00, Ftdi.BitMode.SYNCFF)
    time.sleep(0.1)

    # Set the clock frequency (60 MHz for 480 Mbps)
    ftdi.set_frequency(60000000)

    # Configure GPIO pins (if needed)
    # Example: Set all pins as outputs
    ftdi.set_bitmode(0xFF, Ftdi.BitMode.BITBANG)

    # Enable synchronous FIFO mode
    ftdi.write_data(bytes([0x00]))  # Dummy write to activate FIFO mode

    print("FT2232HQ initialized in synchronous FIFO mode.")
	
def read_data_from_ftdi(ftdi, num_bytes=BUFFER_SIZE):
    """Read data from FT2232H FIFO and store it in a buffer."""
    buffer = bytearray()

    try:
        while len(buffer) < num_bytes:
            data = ftdi.read_data(num_bytes - len(buffer))
            if data:
                buffer.extend(data)
            time.sleep(0.001)  # Small delay to avoid high CPU usage
    except Exception as e:
        print(f"[ERROR] Read error: {e}")

    print(f"[INFO] Read {len(buffer)} bytes from FT2232H")
    return buffer

def generate_angle_grid():
    """Generate azimuth and elevation matrices."""
    azimuth_values = np.linspace(AZIMUTH_RANGE[0], AZIMUTH_RANGE[1], MATRIX_SIZE)
    elevation_values = np.linspace(ELEVATION_RANGE[0], ELEVATION_RANGE[1], MATRIX_SIZE)
    
    azimuth_matrix, elevation_matrix = np.meshgrid(azimuth_values, elevation_values)
    return azimuth_matrix, elevation_matrix

def ca_cfar(matrix):
    """
    Perform CFAR detection on the matrix using vectorized operations.
    """
    detected_targets = np.zeros_like(matrix)

    # Define sliding window
    window_size = 2 * (CFAR_TRAINING_CELLS + CFAR_GUARD_CELLS) + 1
    for i in range(CFAR_TRAINING_CELLS + CFAR_GUARD_CELLS, MATRIX_SIZE - CFAR_TRAINING_CELLS - CFAR_GUARD_CELLS):
        for j in range(CFAR_TRAINING_CELLS + CFAR_GUARD_CELLS, MATRIX_SIZE - CFAR_TRAINING_CELLS - CFAR_GUARD_CELLS):
            # Extract window
            window = matrix[i - CFAR_GUARD_CELLS - CFAR_TRAINING_CELLS : i + CFAR_GUARD_CELLS + CFAR_TRAINING_CELLS + 1,
                            j - CFAR_GUARD_CELLS - CFAR_TRAINING_CELLS : j + CFAR_GUARD_CELLS + CFAR_TRAINING_CELLS + 1]
            
            # Exclude guard cells and CUT
            guard_area = matrix[i - CFAR_GUARD_CELLS : i + CFAR_GUARD_CELLS + 1,
                              j - CFAR_GUARD_CELLS : j + CFAR_GUARD_CELLS + 1]
            training_cells = np.setdiff1d(window, guard_area)

            # Compute noise threshold
            noise_level = np.mean(training_cells)
            threshold = CFAR_THRESHOLD_FACTOR * noise_level

            # Compare CUT against threshold
            if matrix[i, j] > threshold:
                detected_targets[i, j] = matrix[i, j]

    return detected_targets


def process_radar_data(buffer):
    """Convert buffer into an 83x83 matrix, apply CFAR, and find target angles."""
    if len(buffer) != BUFFER_SIZE:
        raise ValueError(f"Invalid buffer size! Expected {BUFFER_SIZE}, got {len(buffer)}")
    
    # Reshape buffer into [83][83] matrix
    matrix = np.array(buffer).reshape((MATRIX_SIZE, MATRIX_SIZE))

    # Apply CFAR
    detected_targets = ca_cfar(matrix)

    # Find position of the max detected target
    max_index = np.unravel_index(np.argmax(detected_targets), matrix.shape)

    # Generate azimuth and elevation mapping
    azimuth_matrix, elevation_matrix = generate_angle_grid()

    # Get azimuth and elevation for detected target
    max_azimuth = azimuth_matrix[max_index]
    max_elevation = elevation_matrix[max_index]

    return detected_targets, max_index, max_azimuth, max_elevation

def plot_target_position(ax, azimuth, elevation):
    """Plot detected targets with CFAR and refresh display."""
    ax.clear()
    ax.set_xlim(AZIMUTH_RANGE)
    ax.set_ylim(ELEVATION_RANGE)
    ax.set_xlabel("Azimuth (°)")
    ax.set_ylabel("Elevation (°)")
    ax.set_title("RADAR Target Detection (CFAR)")

    # Plot the detected target
    ax.scatter(azimuth, elevation, color='red', s=100, label="Detected Target")
    ax.legend()

    plt.draw()
    plt.pause(0.01)  # Pause to update plot

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
    return range_doppler_data

# %%

# Plot range doppler data, loop through at the end of the data set
cmn = ''
i = 0
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
try:
		# Initialize the FTDI device
	ftdi = Ftdi()
	ftdi.open(vendor=VID, product=PID, interface=INTERFACE)
	# Initialize the FT2232HQ for synchronous FIFO mode
	initialize_ft2232hq(ftdi)
	# Initialize plot
        plt.ion()
        fig, ax = plt.subplots(figsize=(6, 6))
	
        while True:
            receive_data() #receive data from FT2232
            all_data = read_data_from_ftdi(ftdi, num_bytes=buf_size)
            buffer = all_data
            # Process the buffer with CFAR
            detected_targets, max_pos, max_azimuth, max_elevation = process_radar_data(buffer)

            # Print detected target angles
            print(f"Detected Target at {max_pos} -> Azimuth: {max_azimuth:.2f}°, Elevation: {max_elevation:.2f}°")

            # Update plot
            plot_target_position(ax, max_azimuth, max_elevation)

            # Refresh delay
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
except KeyboardInterrupt:  # press ctrl-c to stop the loop
    pass
