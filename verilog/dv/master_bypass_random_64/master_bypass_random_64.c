/*
 * SPDX-FileCopyrightText: 2020 Efabless Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * SPDX-License-Identifier: Apache-2.0
 */

// This include is relative to $CARAVEL_PATH (see Makefile)
#include <defs.h>
#include <stub.c>

/*
	MPRJ LA Test:
		- Sets counter clk through LA[64]
		- Sets counter rst through LA[65] 
		- Observes count value for five clk cycle through LA[31:0]
*/

int clk = 0;
int i;

void main()
{
        /* Set up the housekeeping SPI to be connected internally so	*/
	/* that external pin changes don't affect it.			*/

	// reg_spimaster_config = 0xa002;	// Enable, prescaler = 2,
        reg_spi_enable = 1;
                                        // connect to housekeeping SPI

	// Connect the housekeeping SPI to the SPI master
	// so that the CSB line is not left floating.  This allows
	// all of the GPIO pins to be used for user functions.


	// All GPIO pins are configured to be output
	// Used to flad the start/end of a test 

		// Checkbits are management outputs

        reg_mprj_io_31 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_30 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_29 = GPIO_MODE_MGMT_STD_OUTPUT;
        reg_mprj_io_28 = GPIO_MODE_MGMT_STD_OUTPUT;

		// Configure project inputs

		reg_mprj_io_9  = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_cs
		reg_mprj_io_10 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_mosi
		reg_mprj_io_11 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_sclk

		reg_mprj_io_13 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_cs_2
		reg_mprj_io_14 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_mosi_2
		reg_mprj_io_15 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_sclk_2

		reg_mprj_io_17 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_cs_3
		reg_mprj_io_18 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_mosi_3
		reg_mprj_io_19 = GPIO_MODE_USER_STD_INPUT_NOPULL; // minion_sclk_3

		reg_mprj_io_22 = GPIO_MODE_USER_STD_INPUT_NOPULL; // master_miso

		// Configure project outputs

        reg_mprj_io_7  = GPIO_MODE_USER_STD_OUTPUT; // adapter_parity
		reg_mprj_io_8  = GPIO_MODE_USER_STD_OUTPUT; // minion_parity

		reg_mprj_io_12 = GPIO_MODE_USER_STD_OUTPUT; // minion_miso
		reg_mprj_io_16 = GPIO_MODE_USER_STD_OUTPUT; // minion_miso_2
		reg_mprj_io_20 = GPIO_MODE_USER_STD_OUTPUT; // minion_miso_3

		reg_mprj_io_21 = GPIO_MODE_USER_STD_OUTPUT; // master_cs
		reg_mprj_io_23 = GPIO_MODE_USER_STD_OUTPUT; // master_mosi
		reg_mprj_io_24 = GPIO_MODE_USER_STD_OUTPUT; // master_sclk
        

        /* Apply configuration */
        reg_mprj_xfer = 1;
        while (reg_mprj_xfer == 1);

	// Flag start of the test to testbench
	reg_mprj_datal = 0xA0000000;

}
