from pyftdi.ftdi import Ftdi
from pyftdi.usbtools import UsbTools
import time

# FT2232HQ Configuration
VID = 0x0403  # FTDI Vendor ID
PID = 0x6010  # FT2232HQ Product ID
INTERFACE = 1  # Interface A (1) or B (2)

# Buffer to store received data
buffer = bytearray()

def initialize_ft2232hq(ftdi):
    """
    Initialize the FT2232HQ for synchronous FIFO mode.
    """
    # Reset the FT2232HQ
    ftdi.set_bitmode(0x00, Ftdi.BitMode.RESET)

    # Configure the FT2232HQ for synchronous FIFO mode
    ftdi.set_bitmode(0x00, Ftdi.BitMode.SYNCFF)

    # Set the clock frequency (60 MHz for 480 Mbps)
    #ftdi.set_frequency(60000000)

    # Configure GPIO pins (if needed)
    # Example: Set all pins as outputs
    ftdi.set_bitmode(0xFF, Ftdi.BitMode.BITBANG)

    # Enable synchronous FIFO mode
    ftdi.write_data(bytes([0x00]))  # Dummy write to activate FIFO mode

    print("FT2232HQ initialized in synchronous FIFO mode.")

def receive_data():
    try:
        # Initialize the FTDI device
        ftdi = Ftdi()
        ftdi.open(vendor=VID, product=PID, interface=INTERFACE)

        # Initialize the FT2232HQ for synchronous FIFO mode
        initialize_ft2232hq(ftdi)

        # Receive data
        while True:
            # Read data from the FIFO
            data = ftdi.read_data(4096)  # Read up to 4096 bytes at a time
            if data:
                buffer.extend(data)  # Append data to the buffer
                print(f"Received {len(data)} bytes. Total buffer size: {len(buffer)} bytes")
            else:
                print("No data received.")
                break

            # Add a small delay to avoid overwhelming the CPU
            time.sleep(0.001)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the FTDI device
        ftdi.close()
        print("FT2232HQ closed.")

if __name__ == "__main__":
    receive_data()
