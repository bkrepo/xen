/*
 * xen/arch/arm/meson.c
 *
 * Amlogic Meson specific settings
 *
 * Brian Kim <brian.kim@hardkernel.com>
 * Copyright (c) 2016 Hardkernel Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/platform.h>
#include <asm/psci.h>

static const char * const meson_dt_compat[] __initconst =
{
    "amlogic, Gxbb",
    NULL
};

static void meson_system_reset(void)
{
    call_smc(PSCI_0_2_FN_SYSTEM_RESET, 0, 0, 0);
}

static void meson_system_off(void)
{
    call_smc(PSCI_0_2_FN_SYSTEM_OFF, 0, 0, 0);
}

PLATFORM_START(meson, "MESON")
    .compatible = meson_dt_compat,
    .reset      = meson_system_reset,
    .poweroff   = meson_system_off,
PLATFORM_END

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
