# ──────────────────────────────────────────────────────────────────
# settings.py  –  shared configuration for all RPi scripts
# ──────────────────────────────────────────────────────────────────

# STM32 serial connection
# Use the stable by-id path so it survives USB re-plugs
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C068846-if00"
BAUD_RATE   = 115200

# API server  (laptop running Main/api_server.py)
API_IP   = '192.168.41.20'   # ← change to your laptop's IP on the shared Wi-Fi
API_PORT = 5000

# Robot tuning
OUTDOOR_BIG_TURN = False
