freeze("$(BOARD_DIR)", ("pyg.py"))
freeze("$(BOARD_DIR)", ("rfcore_firmware.py", "ble_repl.py"))
freeze("$(BOARD_DIR)", ("neopixel.py"))

include("$(PORT_DIR)/boards/manifest.py")