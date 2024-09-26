
zephyr_library_amend()

# AW9109
zephyr_include_directories(aw9109)
zephyr_library_sources_ifdef(CONFIG_LED_AW9109 aw9109/aw9109.c)

# AW21036
zephyr_include_directories(aw21036)
zephyr_library_sources_ifdef(CONFIG_LED_AW21036 aw21036/aw21036.c)
