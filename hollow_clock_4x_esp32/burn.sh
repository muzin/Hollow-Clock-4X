/Users/sirius/Library/Arduino15/packages/esp32/tools/esptool_py/3.0.0/esptool \
--chip esp32 \
--port "/dev/cu.usbserial-02576431" \
--baud 921600  \
--before default_reset \
--after hard_reset write_flash \
-z --flash_mode dio \
--flash_freq 80m \
--flash_size detect 0xe000 \
"/Users/sirius/Library/Arduino15/packages/esp32/hardware/esp32/1.0.6/tools/partitions/boot_app0.bin" \
0x1000 \
"/Users/sirius/Library/Arduino15/packages/esp32/hardware/esp32/1.0.6/tools/sdk/bin/bootloader_qio_80m.bin" \
0x10000 \
"/Users/sirius/bucket/project/espProjects/hollow_clock_4x/Hollow_Clock_4X/hollow_clock_4x_esp32/build/esp32.esp32.esp32/hollow_clock_4x_esp32.ino.bin" \
0x8000 \
"/Users/sirius/bucket/project/espProjects/hollow_clock_4x/Hollow_Clock_4X/hollow_clock_4x_esp32/build/esp32.esp32.esp32/hollow_clock_4x_esp32.ino.partitions.bin"
