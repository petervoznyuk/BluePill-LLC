# # Compile code and upload it to the Blue Pill
# ~/.platformio/penv/Scripts/platformio.exe run --target upload --environment bluepill_f103c8
# # Start serial monitor to read from and write to the Blue Pill
# # See https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html#cmd-device-monitor-filters
# ~/.platformio/penv/Scripts/platformio.exe device monitor --port COM5 --baud 460800 --eol=LF --filter send_on_enter > data/2024-03-06/$(date +%Y%m%d)_time$(date +%H%M).csv

~/.platformio/penv/Scripts/platformio.exe run --target upload --target monitor --environment bluepill_f103c8