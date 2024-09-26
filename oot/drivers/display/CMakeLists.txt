
zephyr_library_amend()

# SSD1309
zephyr_include_directories(ssd1309)
zephyr_library_sources_ifdef(CONFIG_SSD1309 ssd1309/ssd1309.c)

