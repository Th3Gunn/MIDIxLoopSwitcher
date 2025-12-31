Relay GPIO example (74HC595)
- Wiring: DS->GPIO11, SHCP->GPIO10, STCP->GPIO12
- Button: active-low (CONFIG_BUTTON_PIN, default GPIO8)
- Build: idf.py build && idf.py -p PORT flash monitor
