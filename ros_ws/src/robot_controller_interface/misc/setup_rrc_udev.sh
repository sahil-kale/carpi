#!/bin/bash

# Create udev rule
sudo tee /etc/udev/rules.d/99-rrc.rules > /dev/null <<EOF
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="1a86", ENV{ID_MODEL_ID}=="55d4", ENV{ID_SERIAL_SHORT}=="596F002370", SYMLINK+="rrc"
EOF

# Reload udev and apply rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Wait briefly for udev to apply rule
sleep 1

# Confirm result
if [ -e /dev/rrc ]; then
    echo "✅ /dev/rrc created successfully."
    ls -l /dev/rrc
else
    echo "❌ /dev/rrc not found. Try unplugging and replugging the device."
fi
