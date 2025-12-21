# SPDX-FileCopyrightText: 2025 Aaron White <w531t4@gmail.com>
# SPDX-License-Identifier: MIT
# Portions of this file are based on public-domain (CC0) material.

# Edit following two lines to set component requirements (see docs)
# set(COMPONENT_REQUIRES )
# set(COMPONENT_PRIV_REQUIRES )

# set(COMPONENT_SRCS)
# set(COMPONENT_ADD_INCLUDEDIRS "")
idf_component_register(SRCS "main.cpp" "matrix_panel_fpga.cpp"
                    #    INCLUDE_DIRS "include"
                       PRIV_REQUIRES "esp_driver_spi"
                       PRIV_REQUIRES "esp_driver_gpio")
# register_component()
