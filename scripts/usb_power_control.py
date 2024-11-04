#!/usr/bin/env python
"""Control power for a specific USB device."""

import sys
from pathlib import Path

idVendor = sys.argv[1]
idProduct = sys.argv[2]
port_state = sys.argv[3]
print(f'Looking for {idVendor}:{idProduct}')

if port_state not in ['on', 'off']:
    print('3rd arg port state must be one of "on" or "off"')
    sys.exit(1)

usb_devices = Path('/sys/bus/usb/devices')
for directory in usb_devices.iterdir():
    vendor_path = directory / 'idVendor'
    if not vendor_path.exists():
        continue

    with open(vendor_path, 'r') as f:
        found_vendor = f.read()[:-1]

    if found_vendor != idVendor:
        continue

    product_path = directory / 'idProduct'
    if not product_path.exists():
        continue

    with open(product_path, 'r') as f:
        found_product = f.read()[:-1]

    if found_product != idProduct:
        continue

    with open(directory / 'power' / 'autosuspend_delay_ms', 'w') as f:
        f.write('0\n')

    with open(directory / 'power' / 'control', 'w') as f:
        if port_state == 'off':
            f.write('auto\n')
        else:
            f.write('on\n')

    sys.exit(0)

print('Device not found')
sys.exit(1)
