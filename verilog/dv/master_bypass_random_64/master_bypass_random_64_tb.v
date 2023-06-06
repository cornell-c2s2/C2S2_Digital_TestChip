//========================================================================
// master_bypass_random_64_tb
//========================================================================

`default_nettype none

//------------------------------------------------------------------------
// VTB Helper Macros
//------------------------------------------------------------------------

`timescale 1 ns / 1 ps

`define VTB_INPUT_DELAY 1
`define VTB_OUTPUT_ASSERT_DELAY 3

`define CYCLE_TIME 25
`define INTRA_CYCLE_TIME (`VTB_OUTPUT_ASSERT_DELAY-`VTB_INPUT_DELAY)

`define T(a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17) \
        t(a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,`__LINE__)

// Tick one extra cycle upon an error.
`define VTB_TEST_FAIL(lineno, out, ref, port_name) \
    $display("- Timestamp      : %0d (default unit: ns)", $time); \
    $display("- Cycle number   : %0d (variable: cycle_count)", cycle_count); \
    $display("- line number    : line %0d in master_bypass_random_64_tb.v.cases", lineno); \
    $display("- port name      : %s", port_name); \
    $display("- expected value : 0x%x", ref); \
    $display("- actual value   : 0x%x", out); \
    $display(""); \
    #(`CYCLE_TIME-`INTRA_CYCLE_TIME); \
    cycle_count += 1; \
    #`CYCLE_TIME; \
    cycle_count += 1; \
    $fatal;

`define CHECK(lineno, out, ref, port_name) \
  if ((|(out ^ out)) == 1'b0) ; \
  else begin \
    $display(""); \
    $display("The test bench received a value containing X/Z's! Please note"); \
    $display("that the VTB is pessmistic about X's and you should make sure"); \
    $display("all output ports of your DUT does not produce X's after reset."); \
    `VTB_TEST_FAIL(lineno, out, ref, port_name) \
  end \
  if (out != ref) begin \
    $display(""); \
    $display("The test bench received an incorrect value!"); \
    `VTB_TEST_FAIL(lineno, out, ref, port_name) \
  end

//========================================================================
// Top-Level Test Harness
//========================================================================

module master_bypass_random_64_tb;

  //----------------------------------------------------------------------
  // Create clocks
  //----------------------------------------------------------------------

  // Clock for Caravel

  reg clock = 1'b0;
  always #12.5 clock = ~clock;

  //----------------------------------------------------------------------
  // Instantiate Caravel and SPI Flash
  //----------------------------------------------------------------------

  wire        VDD3V3;
  wire        VDD1V8;
  wire        VSS;
  reg         RSTB;
  reg         CSB;

  wire        gpio;
  wire [37:0] mprj_io;

  wire        flash_csb;
  wire        flash_clk;
  wire        flash_io0;
  wire        flash_io1;

  caravel uut
  (
    .vddio     (VDD3V3),
    .vddio_2   (VDD3V3),
    .vssio     (VSS),
    .vssio_2   (VSS),
    .vdda      (VDD3V3),
    .vssa      (VSS),
    .vccd      (VDD1V8),
    .vssd      (VSS),
    .vdda1     (VDD3V3),
    .vdda1_2   (VDD3V3),
    .vdda2     (VDD3V3),
    .vssa1     (VSS),
    .vssa1_2   (VSS),
    .vssa2     (VSS),
    .vccd1     (VDD1V8),
    .vccd2     (VDD1V8),
    .vssd1     (VSS),
    .vssd2     (VSS),
    .clock     (clock),
    .gpio      (gpio),
    .mprj_io   (mprj_io),
    .flash_csb (flash_csb),
    .flash_clk (flash_clk),
    .flash_io0 (flash_io0),
    .flash_io1 (flash_io1),
    .resetb    (RSTB)
  );

  spiflash
  #(
    .FILENAME ("master_bypass_random_64.hex")
  )
  spiflash
  (
    .csb (flash_csb),
    .clk (flash_clk),
    .io0 (flash_io0),
    .io1 (flash_io1),
    .io2 (),
    .io3 ()
  );

  //----------------------------------------------------------------------
  // Rename the mprj_io
  //----------------------------------------------------------------------

  // Inputs

  logic minion_cs;  
  logic minion_mosi;
  logic minion_sclk; 
  logic minion_cs_2; 
  logic minion_mosi_2;
  logic minion_sclk_2; 
  logic minion_cs_3; 
  logic minion_mosi_3;
  logic minion_sclk_3; 
  logic master_miso;

  assign mprj_io[9]  = minion_cs; 
  assign mprj_io[10] = minion_mosi;
  assign mprj_io[11] = minion_sclk; 

  assign mprj_io[13] = minion_cs_2; 
  assign mprj_io[14] = minion_mosi_2;
  assign mprj_io[15] = minion_sclk_2; 

  assign mprj_io[17] = minion_cs_3; 
  assign mprj_io[18] = minion_mosi_3;
  assign mprj_io[19] = minion_sclk_3; 

  assign mprj_io[22] = master_miso;

  // Outputs

  wire adapter_parity;
  wire minion_parity;
  wire minion_miso;
  wire minion_miso_2;
  wire minion_miso_3;
  wire master_cs;
  wire master_mosi;
  wire master_sclk;

  wire [3:0] checkbits;

  assign adapter_parity = mprj_io[7]; 
  assign minion_parity  = mprj_io[8];

  assign minion_miso    = mprj_io[12];
  assign minion_miso_2  = mprj_io[16];
  assign minion_miso_3  = mprj_io[20];

  assign master_cs      = mprj_io[21]; 
  assign master_mosi    = mprj_io[23]; 
  assign master_sclk    = mprj_io[24];

  assign checkbits      = mprj_io[31:28];

  //----------------------------------------------------------------------
  // Power-up and reset sequence
  //----------------------------------------------------------------------

  initial begin
    RSTB <= 1'b0;
    CSB  <= 1'b1;   // Force CSB high
    #2000;
    RSTB <= 1'b1;   // Release reset
    #300000;
    CSB = 1'b0;     // CSB can be released
  end

  reg power1;
  reg power2;
  reg power3;
  reg power4;

  initial begin
    power1 <= 1'b0;
    power2 <= 1'b0;
    power3 <= 1'b0;
    power4 <= 1'b0;
    #100;
    power1 <= 1'b1;
    #100;
    power2 <= 1'b1;
    #100;
    power3 <= 1'b1;
    #100;
    power4 <= 1'b1;
  end

  assign VDD3V3 = power1;
  assign VDD1V8 = power2;
  assign VSS    = 1'b0;

  //----------------------------------------------------------------------
  // Setup VCD dumping and overall timeout
  //----------------------------------------------------------------------

  initial begin
    $dumpfile("master_bypass_random_64.vcd");
    $dumpvars(0, master_bypass_random_64_tb);
    #1;

    // Repeat cycles of 1000 clock edges as needed to complete testbench
    repeat (75) begin
      repeat (1000) @(posedge clock);
    end
    $display("%c[1;31m",27);
    `ifdef GL
      $display ("Monitor: Timeout GL Failed");
    `else
      $display ("Monitor: Timeout RTL Failed");
    `endif
    $display("%c[0m",27);
    $finish;
  end

  //----------------------------------------------------------------------
  // VTB task for assigning inputs and checking outputs
  //----------------------------------------------------------------------

  integer cycle_count;

  task t(
    input logic [0:0] ref_adapter_parity,
    input logic [0:0] ref_minion_parity,
    input logic [0:0] inp_minion_cs,
    input logic [0:0] inp_minion_cs_2,
    input logic [0:0] inp_minion_cs_3,
    input logic [0:0] ref_minion_miso,
    input logic [0:0] ref_minion_miso_2,
    input logic [0:0] ref_minion_miso_3,
    input logic [0:0] inp_minion_mosi,
    input logic [0:0] inp_minion_mosi_2,
    input logic [0:0] inp_minion_mosi_3,
    input logic [0:0] ref_master_cs,
    input logic [0:0] inp_master_miso,
    input logic [0:0] ref_master_mosi,
    input logic [0:0] ref_master_sclk,
    input logic [0:0] inp_minion_sclk,
    input logic [0:0] inp_minion_sclk_2,
    input logic [0:0] inp_minion_sclk_3,
    input integer lineno
  );
  begin

    // Set the inputs 

    minion_cs     = inp_minion_cs;
    minion_cs_2   = inp_minion_cs_2;
    minion_cs_3   = inp_minion_cs_3;
    minion_mosi   = inp_minion_mosi;
    minion_mosi_2 = inp_minion_mosi_2;
    minion_mosi_3 = inp_minion_mosi_3;
    master_miso   = inp_master_miso;
    minion_sclk   = inp_minion_sclk;
    minion_sclk_2 = inp_minion_sclk_2;
    minion_sclk_3 = inp_minion_sclk_3;

    // Check the outputs

    #`INTRA_CYCLE_TIME;
    `CHECK(lineno, adapter_parity, ref_adapter_parity, "adapter_parity  (adapter_parity in Verilog)");
    `CHECK(lineno, minion_parity,  ref_minion_parity,  "minion_parity   (minion_parity in Verilog)");
    `CHECK(lineno, minion_miso,    ref_minion_miso,    "spi_min.miso    (minion_miso in Verilog)");
    `CHECK(lineno, minion_miso_2,  ref_minion_miso_2,  "spi_min.miso_2  (minion_miso_2 in Verilog)");
    `CHECK(lineno, minion_miso_3,  ref_minion_miso_3,  "spi_min.miso_3  (minion_miso_3 in Verilog)");
    `CHECK(lineno, master_cs,      ref_master_cs,      "spi_min.ms_cs   (master_cs in Verilog)");
    `CHECK(lineno, master_mosi,    ref_master_mosi,    "spi_min.ms_mosi (master_mosi in Verilog)");
    `CHECK(lineno, master_sclk,    ref_master_sclk,    "spi_min.ms_sclk (master_sclk in Verilog)");

    // Advance to the next cycle
    
    #(`CYCLE_TIME-`INTRA_CYCLE_TIME);
    cycle_count += 1;
  end
  endtask

  //----------------------------------------------------------------------
  // Execute the generated VTB cases
  //----------------------------------------------------------------------

  initial begin

    // This is how we wait for the firmware to configure the IO ports
    wait (checkbits == 4'hA);

    wait (clock == 1);
    wait (clock == 0);
    cycle_count = 0;

    #(`CYCLE_TIME/2);

    #`VTB_INPUT_DELAY;
    #`CYCLE_TIME;
    cycle_count = 1;
    #`CYCLE_TIME;
    cycle_count = 2;
    // 2 cycles plus input delay

    // Start test
    `include "master_bypass_random_64_tb.v.cases"

    $display("");
    $display("  [ passed ]");
    $display("");

    // Tick one extra cycle for better waveform
    #`CYCLE_TIME;
    cycle_count += 1;
    $finish;

  end

endmodule

`default_nettype wire
