/**
 * Copyright (c) 2015-2016 Nuand, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef NLMS7_NUAND_REGS_INCLUDED_H_
#define NLMS7_NUAND_REGS_INCLUDED_H_

#include <stdint.h>

/* This file defines the Avalon-MM register space for the Nuand LMS7 controller.
 * See hdl/doc/memory_map.txt for the complete memory map.
 */

/* Select the Nuand memory map that's overlaid atop unused LMS7 address space */
#define NLMS7_SEL_NUAND_REGION  (1 << 12)

/* Address functions */
#define NLMS7_FUNC_JESD1        (0x0 << 4)  /* JESD207 Control - Port 1 */
#define NLMS7_FUNC_JESD2        (0x1 << 4)  /* JESD207 Control - Port 2 */
#define NLMS7_FUNC_GPOUT        (0x2 << 4)  /* General purpose output   */
#define NLMS7_FUNC_GPIN         (0x3 << 4)  /* General purpose input    */
#define NLMS7_FUNC_VER          (0x4 << 4)  /* Version information */

/* Macro to build register addresses */
#define NLMS7_REG_ADDR(func, subaddr) \
            (NLMS7_SEL_NUAND_REGION | NLMS7_FUNC_##func | subaddr)

/******************************************************************************
 * JESD207 Control
 *****************************************************************************/

/* Port 1 & 2 Control register addresses */
#define JESDP1_CTRL_ADDR    NLMS7_REG_ADDR(JESD1, 0x0)
#define JESDP2_CTRL_ADDR    NLMS7_REG_ADDR(JESD2, 0x0)

/* Start/stop bit values */
#define JESD_CTRL_START     (1 << 0)
#define JESD_CTRL_STOP      (0 << 0)

/* Direction configuration */
#define JESD_CTRL_DIR_RX    (0 << 1)
#define JESD_CTRL_DIR_TX    (1 << 1)

/******************************************************************************
 * General purpose output
 *****************************************************************************/

/* General output control register */
#define GPOUT_CTRL_ADDR     NLMS7_REG_ADDR(GPOUT, 0x0)

/* LMS7002M hardware reset (active low) */
#define GPOUT_CTRL_LMS7_RESETN      (1 << 0)

/* Core LDO enable bit (active high) */
#define GPOUT_CTRL_CORE_LDO_ENABLE  (1 << 4)

/******************************************************************************
 * IP core version info
 *****************************************************************************/

#define VER_MAJOR_ADDR  NLMS7_REG_ADDR(VER, 0x0)
#define VER_MINOR_ADDR  NLMS7_REG_ADDR(VER, 0x1)
#define VER_PATCH_ADDR  NLMS7_REG_ADDR(VER, 0x2)

#endif
