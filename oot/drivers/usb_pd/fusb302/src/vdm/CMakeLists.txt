zephyr_library()
zephyr_include_directories(.)
zephyr_library_sources(
    bitfield_translators.c
    vdm.c
    vdm_callbacks.c
)
add_subdirectory(DisplayPort)

