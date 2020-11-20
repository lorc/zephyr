# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_ARM_MMU mmu_regions.c)

# zephyr_sources_ifdef(CONFIG_SOC_R8A7795_A72    plat_core.S)

