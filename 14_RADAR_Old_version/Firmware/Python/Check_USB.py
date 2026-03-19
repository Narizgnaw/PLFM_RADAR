import usb.core
import usb.backend.libusb1

backend = usb.backend.libusb1.get_backend(find_library=lambda x: "C:/Windows/System32/libusb-1.0.dll")
devices = usb.core.find(find_all=True, backend=backend)

print("USB devices found:", list(devices))
