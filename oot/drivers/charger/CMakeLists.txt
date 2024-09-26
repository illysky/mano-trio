
zephyr_library_amend()
zephyr_include_directories(bq25792)
zephyr_library_sources_ifdef(CONFIG_CHARGER_BQ25792 bq25792/bq25792.c)

