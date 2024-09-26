if(CONFIG_FUSB302)  
    zephyr_library()
    zephyr_include_directories(.)
    zephyr_library_sources(
        usb_pd.c
    )
    add_subdirectory(src)
endif()