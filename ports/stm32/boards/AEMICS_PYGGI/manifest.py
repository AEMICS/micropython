freeze("$(BOARD_DIR)", ("pyg.py"))
freeze("$(BOARD_DIR)", ("rfcore_firmware.py", "ble_repl.py"))

include("$(PORT_DIR)/boards/manifest.py")
