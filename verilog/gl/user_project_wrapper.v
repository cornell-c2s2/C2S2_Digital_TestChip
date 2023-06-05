module user_project_wrapper (user_clock2,
    vccd1,
    vccd2,
    vdda1,
    vdda2,
    vssa1,
    vssa2,
    vssd1,
    vssd2,
    wb_clk_i,
    wb_rst_i,
    wbs_ack_o,
    wbs_cyc_i,
    wbs_stb_i,
    wbs_we_i,
    analog_io,
    io_in,
    io_oeb,
    io_out,
    la_data_in,
    la_data_out,
    la_oenb,
    user_irq,
    wbs_adr_i,
    wbs_dat_i,
    wbs_dat_o,
    wbs_sel_i);
 input user_clock2;
 input vccd1;
 input vccd2;
 input vdda1;
 input vdda2;
 input vssa1;
 input vssa2;
 input vssd1;
 input vssd2;
 input wb_clk_i;
 input wb_rst_i;
 output wbs_ack_o;
 input wbs_cyc_i;
 input wbs_stb_i;
 input wbs_we_i;
 inout [28:0] analog_io;
 input [37:0] io_in;
 output [37:0] io_oeb;
 output [37:0] io_out;
 input [127:0] la_data_in;
 output [127:0] la_data_out;
 input [127:0] la_oenb;
 output [2:0] user_irq;
 input [31:0] wbs_adr_i;
 input [31:0] wbs_dat_i;
 output [31:0] wbs_dat_o;
 input [3:0] wbs_sel_i;

 wire _0_;

 FFTSPIInterconnectRTL fft_spi (.adapter_parity(io_out[5]),
    .clk(wb_clk_i),
    .master_cs(io_out[19]),
    .master_miso(io_out[20]),
    .master_mosi(io_out[21]),
    .master_sclk(io_in[22]),
    .minion_cs(io_in[7]),
    .minion_cs_2(io_in[11]),
    .minion_cs_3(io_in[15]),
    .minion_miso(io_out[10]),
    .minion_miso_2(io_out[14]),
    .minion_miso_3(io_out[18]),
    .minion_mosi(io_in[8]),
    .minion_mosi_2(io_in[12]),
    .minion_mosi_3(io_in[16]),
    .minion_parity(io_out[6]),
    .minion_sclk(io_in[9]),
    .minion_sclk_2(io_in[13]),
    .minion_sclk_3(io_in[17]),
    .reset(wb_rst_i),
    .vccd1(vccd1),
    .vssd1(vssd1),
    .io_oeb({_0_,
    io_oeb[22],
    io_oeb[21],
    io_oeb[20],
    io_oeb[19],
    io_oeb[18],
    io_oeb[17],
    io_oeb[16],
    io_oeb[15],
    io_oeb[14],
    io_oeb[13],
    io_oeb[12],
    io_oeb[11],
    io_oeb[10],
    io_oeb[9],
    io_oeb[8],
    io_oeb[7],
    io_oeb[6],
    io_oeb[5]}));
endmodule
