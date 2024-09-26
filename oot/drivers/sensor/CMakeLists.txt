
zephyr_library_amend()

# HX711
zephyr_include_directories(hx711)
zephyr_library_sources_ifdef(CONFIG_SENSOR_HX711 hx711/hx711.c)

# MXC6655XA
zephyr_include_directories(mxc6655xa)
zephyr_library_sources_ifdef(CONFIG_SENSOR_MXC6655XA mxc6655xa/mxc6655xa.c)

# PAA5100
zephyr_include_directories(paa5100)
zephyr_library_sources_ifdef(CONFIG_SENSOR_PAA5100 paa5100/paa5100.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_PAA5100 paa5100/paa5100_priv.c)

# VCNL4010
zephyr_include_directories(vcnl4010)
zephyr_library_sources_ifdef(CONFIG_SENSOR_PAA5100 vcnl4010/vcnl4010.c)
