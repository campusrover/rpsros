import pyudev

context = pyudev.Context()

for device in context.list_devices(subsystem='usb'):
    vid = device.attributes.get('idVendor')
    pid = device.attributes.get('idProduct')
    
    if vid and pid:
        print(f"Vendor ID: {vid}, Product ID: {pid}")