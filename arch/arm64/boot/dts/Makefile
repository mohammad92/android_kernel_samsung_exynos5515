# SPDX-License-Identifier: GPL-2.0
dtb-y += exynos/s5e5515.dtb
dtbo-y += samsung/exynos5515-freshul_na_open_00.dtbo
dtbo-y += samsung/exynos5515-freshul_na_open_01.dtbo
dtbo-y += samsung/exynos5515-freshul_na_open_03.dtbo
dtbo-y += samsung/exynos5515-freshul_na_open_05.dtbo
dtbo-y += samsung/exynos5515-freshul_na_open_15.dtbo

targets += dtbs
DTB_LIST  := $(dtb-y) $(dtbo-y)
always    := $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := *.dtb*
