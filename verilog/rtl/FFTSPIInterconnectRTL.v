module SPI_v3_components_ShiftReg (
	clk,
	in_,
	load_data,
	load_en,
	out,
	reset,
	shift_en
);
	parameter nbits = 8;
	parameter reset_value = 1'b0;
	input wire clk;
	input wire in_;
	input wire [nbits - 1:0] load_data;
	input wire load_en;
	output reg [nbits - 1:0] out;
	input wire reset;
	input wire shift_en;
	always @(posedge clk)
		if (reset)
			out <= {nbits {reset_value}};
		else if (load_en)
			out <= load_data;
		else if (~load_en & shift_en)
			out <= {out[nbits - 2:0], in_};
endmodule
module vc_Reg (
	clk,
	q,
	d
);
	parameter p_nbits = 1;
	input wire clk;
	output reg [p_nbits - 1:0] q;
	input wire [p_nbits - 1:0] d;
	always @(posedge clk) q <= d;
endmodule
module vc_ResetReg (
	clk,
	reset,
	q,
	d
);
	parameter p_nbits = 1;
	parameter p_reset_value = 0;
	input wire clk;
	input wire reset;
	output reg [p_nbits - 1:0] q;
	input wire [p_nbits - 1:0] d;
	always @(posedge clk) q <= (reset ? p_reset_value : d);
endmodule
module vc_EnReg (
	clk,
	reset,
	q,
	d,
	en
);
	parameter p_nbits = 1;
	input wire clk;
	input wire reset;
	output reg [p_nbits - 1:0] q;
	input wire [p_nbits - 1:0] d;
	input wire en;
	always @(posedge clk)
		if (en)
			q <= d;
endmodule
module vc_EnResetReg (
	clk,
	reset,
	q,
	d,
	en
);
	parameter p_nbits = 1;
	parameter p_reset_value = 0;
	input wire clk;
	input wire reset;
	output reg [p_nbits - 1:0] q;
	input wire [p_nbits - 1:0] d;
	input wire en;
	always @(posedge clk)
		if (reset || en)
			q <= (reset ? p_reset_value : d);
endmodule
module SPI_minion_components_Synchronizer (
	clk,
	in_,
	negedge_,
	out,
	posedge_,
	reset
);
	parameter reset_value = 1'b0;
	input wire clk;
	input wire in_;
	output reg negedge_;
	output wire out;
	output reg posedge_;
	input wire reset;
	reg [2:0] shreg;
	always @(*) begin
		negedge_ = shreg[2] & ~shreg[1];
		posedge_ = ~shreg[2] & shreg[1];
	end
	always @(posedge clk)
		if (reset)
			shreg <= {3 {reset_value}};
		else
			shreg <= {shreg[1:0], in_};
	assign out = shreg[1];
endmodule
module SPIMasterValRdyVRTL (
	clk,
	reset,
	spi_ifc_cs,
	spi_ifc_miso,
	spi_ifc_mosi,
	spi_ifc_sclk,
	recv_val,
	recv_rdy,
	recv_msg,
	send_val,
	send_rdy,
	send_msg,
	packet_size_ifc_val,
	packet_size_ifc_rdy,
	packet_size_ifc_msg,
	cs_addr_ifc_val,
	cs_addr_ifc_rdy,
	cs_addr_ifc_msg,
	freq_ifc_val,
	freq_ifc_rdy,
	freq_ifc_msg
);
	parameter nbits = 34;
	parameter ncs = 1;
	parameter logBitsN = $clog2(nbits) + 1;
	parameter logCSN = (ncs > 1 ? $clog2(ncs) : 1);
	input wire clk;
	input wire reset;
	output reg [0:ncs - 1] spi_ifc_cs;
	input wire spi_ifc_miso;
	output wire spi_ifc_mosi;
	output reg spi_ifc_sclk;
	input wire recv_val;
	output reg recv_rdy;
	input wire [nbits - 1:0] recv_msg;
	output reg send_val;
	input wire send_rdy;
	output wire [nbits - 1:0] send_msg;
	input wire packet_size_ifc_val;
	output wire packet_size_ifc_rdy;
	input wire [logBitsN - 1:0] packet_size_ifc_msg;
	input wire cs_addr_ifc_val;
	output wire cs_addr_ifc_rdy;
	input wire [logCSN - 1:0] cs_addr_ifc_msg;
	input wire freq_ifc_val;
	output wire freq_ifc_rdy;
	input wire [2:0] freq_ifc_msg;
	wire [logBitsN - 1:0] packet_size_reg_out;
	reg packet_size_reg_en;
	wire [logCSN - 1:0] cs_addr_reg_out;
	reg cs_addr_reg_en;
	reg [logBitsN - 1:0] sclk_counter;
	reg sclk_counter_en;
	reg [6:0] freq_high_counter;
	reg freq_high_counter_en;
	reg [6:0] freq_low_counter;
	reg freq_low_counter_en;
	wire [2:0] freq_reg_out;
	reg freq_reg_en;
	wire [nbits - 1:0] shreg_in_out;
	wire [nbits - 1:0] shreg_out_out;
	reg freq_high_refill;
	reg freq_low_refill;
	wire miso_sync_clk;
	wire miso_sync_in_;
	wire miso_sync_negedge_;
	wire miso_sync_out;
	wire miso_sync_posedge_;
	wire miso_sync_reset;
	SPI_minion_components_Synchronizer #(.reset_value(1'b0)) miso_sync(
		.clk(miso_sync_clk),
		.in_(miso_sync_in_),
		.negedge_(miso_sync_negedge_),
		.out(miso_sync_out),
		.posedge_(miso_sync_posedge_),
		.reset(miso_sync_reset)
	);
	assign miso_sync_clk = clk;
	assign miso_sync_reset = reset;
	assign miso_sync_in_ = spi_ifc_miso;
	vc_EnResetReg #(.p_nbits(logBitsN)) packet_size_reg(
		.clk(clk),
		.reset(reset),
		.q(packet_size_reg_out),
		.d(packet_size_ifc_msg),
		.en(packet_size_reg_en)
	);
	vc_EnResetReg #(.p_nbits(logCSN)) cs_addr_reg(
		.clk(clk),
		.reset(reset),
		.q(cs_addr_reg_out),
		.d(cs_addr_ifc_msg),
		.en(cs_addr_reg_en)
	);
	vc_EnResetReg #(.p_nbits(3)) freq_reg(
		.clk(clk),
		.reset(reset),
		.q(freq_reg_out),
		.d(freq_ifc_msg),
		.en(freq_reg_en)
	);
	assign packet_size_ifc_rdy = recv_rdy;
	assign cs_addr_ifc_rdy = recv_rdy;
	assign freq_ifc_rdy = recv_rdy;
	reg sclk_negedge;
	reg sclk_posedge;
	reg shreg_out_rst;
	reg [3:0] state;
	reg [3:0] next_state;
	always @(posedge clk) begin : up_state
		if (reset)
			state <= 4'd0;
		else
			state <= next_state;
	end
	always @(*) begin : up_stateChange
		case (state)
			4'd0: next_state = (recv_val ? 4'd1 : 4'd0);
			4'd1: next_state = 4'd2;
			4'd2: next_state = 4'd3;
			4'd3:
				if (freq_high_counter == 0)
					next_state = 4'd4;
				else
					next_state = 4'd5;
			4'd4:
				if (freq_low_counter == 0)
					next_state = (sclk_counter == 0 ? 4'd7 : 4'd3);
				else
					next_state = 4'd6;
			4'd5:
				if (freq_high_counter == 0)
					next_state = 4'd4;
				else
					next_state = 4'd5;
			4'd6:
				if (freq_low_counter == 0)
					next_state = (sclk_counter == 0 ? 4'd7 : 4'd3);
				else
					next_state = 4'd6;
			4'd7: next_state = 4'd8;
			4'd8:
				if (recv_val)
					next_state = 4'd1;
				else if (send_rdy)
					next_state = 4'd0;
				else
					next_state = 4'd8;
			default: next_state = 4'd0;
		endcase
	end
	always @(*) begin : up_stateOutputs
		recv_rdy = 0;
		send_val = 0;
		spi_ifc_sclk = 0;
		packet_size_reg_en = 0;
		cs_addr_reg_en = 0;
		begin : sv2v_autoblock_1
			integer i;
			for (i = 0; i < ncs; i = i + 1)
				spi_ifc_cs[i] = 1;
		end
		sclk_negedge = 0;
		sclk_posedge = 0;
		sclk_counter_en = 0;
		shreg_out_rst = 0;
		freq_high_refill = 0;
		freq_low_refill = 0;
		freq_high_counter_en = 0;
		freq_low_counter_en = 0;
		if (state == 4'd0) begin
			recv_rdy = 1;
			packet_size_reg_en = packet_size_ifc_val;
			cs_addr_reg_en = cs_addr_ifc_val;
			freq_reg_en = freq_ifc_val;
		end
		else if (state == 4'd1) begin
			spi_ifc_cs[cs_addr_reg_out] = 0;
			shreg_out_rst = 1;
		end
		else if (state == 4'd2) begin
			sclk_posedge = 1;
			spi_ifc_cs[cs_addr_reg_out] = 0;
		end
		else if (state == 4'd3) begin
			spi_ifc_cs[cs_addr_reg_out] = 0;
			spi_ifc_sclk = 1;
			sclk_negedge = freq_high_counter == 0;
			sclk_counter_en = 1;
			freq_high_counter_en = 1;
			freq_low_refill = 1;
		end
		else if (state == 4'd4) begin
			sclk_posedge = (sclk_counter != 0) && (freq_low_counter == 0);
			spi_ifc_cs[cs_addr_reg_out] = 0;
			spi_ifc_sclk = 0;
			freq_low_counter_en = 1;
			freq_high_refill = 1;
		end
		else if (state == 4'd5) begin
			spi_ifc_cs[cs_addr_reg_out] = 0;
			spi_ifc_sclk = 1;
			sclk_negedge = freq_high_counter == 0;
			sclk_counter_en = 0;
			freq_high_counter_en = 1;
		end
		else if (state == 4'd6) begin
			sclk_posedge = (sclk_counter != 0) && (freq_low_counter == 0);
			spi_ifc_cs[cs_addr_reg_out] = 0;
			spi_ifc_sclk = 0;
			freq_low_counter_en = 1;
		end
		else if (state == 4'd7)
			spi_ifc_cs[cs_addr_reg_out] = 0;
		else if (state == 4'd8) begin
			recv_rdy = 1;
			send_val = 1;
			packet_size_reg_en = packet_size_ifc_val;
			cs_addr_reg_en = cs_addr_ifc_val;
			freq_reg_en = freq_ifc_val;
		end
	end
	always @(posedge clk)
		if (reset)
			sclk_counter <= 0;
		else if (recv_val & recv_rdy)
			sclk_counter <= packet_size_reg_out;
		else if (sclk_counter_en)
			sclk_counter <= sclk_counter - 1;
	always @(posedge clk)
		if (reset)
			freq_high_counter <= 0;
		else if ((recv_val & recv_rdy) | freq_high_refill)
			freq_high_counter <= (2 ** freq_reg_out) - 1;
		else if (freq_high_counter_en)
			freq_high_counter <= freq_high_counter - 1;
	always @(posedge clk)
		if (reset)
			freq_low_counter <= 0;
		else if ((recv_val & recv_rdy) | freq_low_refill)
			freq_low_counter <= (2 ** freq_reg_out) - 1;
		else if (freq_low_counter_en)
			freq_low_counter <= freq_low_counter - 1;
	SPI_v3_components_ShiftReg #(
		.nbits(nbits),
		.reset_value(1'b0)
	) shreg_in(
		.clk(clk),
		.in_(miso_sync_out),
		.load_data(0),
		.load_en(0),
		.out(shreg_in_out),
		.reset(shreg_out_rst),
		.shift_en(sclk_posedge)
	);
	SPI_v3_components_ShiftReg #(
		.nbits(nbits),
		.reset_value(1'b0)
	) shreg_out(
		.clk(clk),
		.in_(0),
		.load_data(recv_msg << (nbits - packet_size_reg_out)),
		.load_en(recv_rdy & recv_val),
		.out(shreg_out_out),
		.reset(reset),
		.shift_en(sclk_negedge)
	);
	assign spi_ifc_mosi = shreg_out_out[nbits - 1];
	assign send_msg = shreg_in_out;
endmodule
module SPI_minion_components_ShiftReg (
	clk,
	in_,
	load_data,
	load_en,
	out,
	reset,
	shift_en
);
	parameter nbits = 8;
	parameter reset_value = 1'b0;
	input wire clk;
	input wire in_;
	input wire [nbits - 1:0] load_data;
	input wire load_en;
	output reg [nbits - 1:0] out;
	input wire reset;
	input wire shift_en;
	always @(posedge clk)
		if (reset)
			out <= {nbits {reset_value}};
		else if (load_en)
			out <= load_data;
		else if (~load_en & shift_en)
			out <= {out[nbits - 2:0], in_};
endmodule
module SPI_minion_components_SPIMinionVRTL (
	clk,
	cs,
	miso,
	mosi,
	reset,
	sclk,
	pull_en,
	pull_msg,
	push_en,
	push_msg,
	parity
);
	parameter nbits = 8;
	input wire clk;
	input wire cs;
	output wire miso;
	input wire mosi;
	input wire reset;
	input wire sclk;
	output wire pull_en;
	input wire [nbits - 1:0] pull_msg;
	output wire push_en;
	output wire [nbits - 1:0] push_msg;
	output wire parity;
	wire cs_sync_clk;
	wire cs_sync_in_;
	wire cs_sync_negedge_;
	wire cs_sync_out;
	wire cs_sync_posedge_;
	wire cs_sync_reset;
	SPI_minion_components_Synchronizer #(.reset_value(1'b1)) cs_sync(
		.clk(cs_sync_clk),
		.in_(cs_sync_in_),
		.negedge_(cs_sync_negedge_),
		.out(cs_sync_out),
		.posedge_(cs_sync_posedge_),
		.reset(cs_sync_reset)
	);
	wire mosi_sync_clk;
	wire mosi_sync_in_;
	wire mosi_sync_negedge_;
	wire mosi_sync_out;
	wire mosi_sync_posedge_;
	wire mosi_sync_reset;
	SPI_minion_components_Synchronizer #(.reset_value(1'b0)) mosi_sync(
		.clk(mosi_sync_clk),
		.in_(mosi_sync_in_),
		.negedge_(mosi_sync_negedge_),
		.out(mosi_sync_out),
		.posedge_(mosi_sync_posedge_),
		.reset(mosi_sync_reset)
	);
	wire sclk_sync_clk;
	wire sclk_sync_in_;
	wire sclk_sync_negedge_;
	wire sclk_sync_out;
	wire sclk_sync_posedge_;
	wire sclk_sync_reset;
	SPI_minion_components_Synchronizer #(.reset_value(1'b0)) sclk_sync(
		.clk(sclk_sync_clk),
		.in_(sclk_sync_in_),
		.negedge_(sclk_sync_negedge_),
		.out(sclk_sync_out),
		.posedge_(sclk_sync_posedge_),
		.reset(sclk_sync_reset)
	);
	wire shreg_in_clk;
	wire shreg_in_in_;
	wire [nbits - 1:0] shreg_in_load_data;
	wire shreg_in_load_en;
	wire [nbits - 1:0] shreg_in_out;
	wire shreg_in_reset;
	reg shreg_in_shift_en;
	SPI_minion_components_ShiftReg #(.nbits(nbits)) shreg_in(
		.clk(shreg_in_clk),
		.in_(shreg_in_in_),
		.load_data(shreg_in_load_data),
		.load_en(shreg_in_load_en),
		.out(shreg_in_out),
		.reset(shreg_in_reset),
		.shift_en(shreg_in_shift_en)
	);
	wire shreg_out_clk;
	wire shreg_out_in_;
	wire [nbits - 1:0] shreg_out_load_data;
	wire shreg_out_load_en;
	wire [nbits - 1:0] shreg_out_out;
	wire shreg_out_reset;
	reg shreg_out_shift_en;
	SPI_minion_components_ShiftReg #(.nbits(nbits)) shreg_out(
		.clk(shreg_out_clk),
		.in_(shreg_out_in_),
		.load_data(shreg_out_load_data),
		.load_en(shreg_out_load_en),
		.out(shreg_out_out),
		.reset(shreg_out_reset),
		.shift_en(shreg_out_shift_en)
	);
	always @(*) begin
		shreg_in_shift_en = ~cs_sync_out & sclk_sync_posedge_;
		shreg_out_shift_en = ~cs_sync_out & sclk_sync_negedge_;
	end
	assign cs_sync_clk = clk;
	assign cs_sync_reset = reset;
	assign cs_sync_in_ = cs;
	assign sclk_sync_clk = clk;
	assign sclk_sync_reset = reset;
	assign sclk_sync_in_ = sclk;
	assign mosi_sync_clk = clk;
	assign mosi_sync_reset = reset;
	assign mosi_sync_in_ = mosi;
	assign shreg_in_clk = clk;
	assign shreg_in_reset = reset;
	assign shreg_in_in_ = mosi_sync_out;
	assign shreg_in_load_en = 1'b0;
	assign shreg_in_load_data = {nbits {1'b0}};
	assign shreg_out_clk = clk;
	assign shreg_out_reset = reset;
	assign shreg_out_in_ = 1'b0;
	assign shreg_out_load_en = pull_en;
	assign shreg_out_load_data = pull_msg;
	assign miso = shreg_out_out[nbits - 1];
	assign pull_en = cs_sync_negedge_;
	assign push_en = cs_sync_posedge_;
	assign push_msg = shreg_in_out;
	assign parity = ^push_msg[nbits - 3:0] & push_en;
endmodule
module vc_Mux2 (
	in0,
	in1,
	sel,
	out
);
	parameter p_nbits = 1;
	input wire [p_nbits - 1:0] in0;
	input wire [p_nbits - 1:0] in1;
	input wire sel;
	output reg [p_nbits - 1:0] out;
	always @(*)
		case (sel)
			1'd0: out = in0;
			1'd1: out = in1;
			default: out = {p_nbits {1'bx}};
		endcase
endmodule
module vc_Mux3 (
	in0,
	in1,
	in2,
	sel,
	out
);
	parameter p_nbits = 1;
	input wire [p_nbits - 1:0] in0;
	input wire [p_nbits - 1:0] in1;
	input wire [p_nbits - 1:0] in2;
	input wire [1:0] sel;
	output reg [p_nbits - 1:0] out;
	always @(*)
		case (sel)
			2'd0: out = in0;
			2'd1: out = in1;
			2'd2: out = in2;
			default: out = {p_nbits {1'bx}};
		endcase
endmodule
module vc_Mux4 (
	in0,
	in1,
	in2,
	in3,
	sel,
	out
);
	parameter p_nbits = 1;
	input wire [p_nbits - 1:0] in0;
	input wire [p_nbits - 1:0] in1;
	input wire [p_nbits - 1:0] in2;
	input wire [p_nbits - 1:0] in3;
	input wire [1:0] sel;
	output reg [p_nbits - 1:0] out;
	always @(*)
		case (sel)
			2'd0: out = in0;
			2'd1: out = in1;
			2'd2: out = in2;
			2'd3: out = in3;
			default: out = {p_nbits {1'bx}};
		endcase
endmodule
module vc_Mux5 (
	in0,
	in1,
	in2,
	in3,
	in4,
	sel,
	out
);
	parameter p_nbits = 1;
	input wire [p_nbits - 1:0] in0;
	input wire [p_nbits - 1:0] in1;
	input wire [p_nbits - 1:0] in2;
	input wire [p_nbits - 1:0] in3;
	input wire [p_nbits - 1:0] in4;
	input wire [2:0] sel;
	output reg [p_nbits - 1:0] out;
	always @(*)
		case (sel)
			3'd0: out = in0;
			3'd1: out = in1;
			3'd2: out = in2;
			3'd3: out = in3;
			3'd4: out = in4;
			default: out = {p_nbits {1'bx}};
		endcase
endmodule
module vc_Mux6 (
	in0,
	in1,
	in2,
	in3,
	in4,
	in5,
	sel,
	out
);
	parameter p_nbits = 1;
	input wire [p_nbits - 1:0] in0;
	input wire [p_nbits - 1:0] in1;
	input wire [p_nbits - 1:0] in2;
	input wire [p_nbits - 1:0] in3;
	input wire [p_nbits - 1:0] in4;
	input wire [p_nbits - 1:0] in5;
	input wire [2:0] sel;
	output reg [p_nbits - 1:0] out;
	always @(*)
		case (sel)
			3'd0: out = in0;
			3'd1: out = in1;
			3'd2: out = in2;
			3'd3: out = in3;
			3'd4: out = in4;
			3'd5: out = in5;
			default: out = {p_nbits {1'bx}};
		endcase
endmodule
module vc_Mux7 (
	in0,
	in1,
	in2,
	in3,
	in4,
	in5,
	in6,
	sel,
	out
);
	parameter p_nbits = 1;
	input wire [p_nbits - 1:0] in0;
	input wire [p_nbits - 1:0] in1;
	input wire [p_nbits - 1:0] in2;
	input wire [p_nbits - 1:0] in3;
	input wire [p_nbits - 1:0] in4;
	input wire [p_nbits - 1:0] in5;
	input wire [p_nbits - 1:0] in6;
	input wire [2:0] sel;
	output reg [p_nbits - 1:0] out;
	always @(*)
		case (sel)
			3'd0: out = in0;
			3'd1: out = in1;
			3'd2: out = in2;
			3'd3: out = in3;
			3'd4: out = in4;
			3'd5: out = in5;
			3'd6: out = in6;
			default: out = {p_nbits {1'bx}};
		endcase
endmodule
module vc_Mux8 (
	in0,
	in1,
	in2,
	in3,
	in4,
	in5,
	in6,
	in7,
	sel,
	out
);
	parameter p_nbits = 1;
	input wire [p_nbits - 1:0] in0;
	input wire [p_nbits - 1:0] in1;
	input wire [p_nbits - 1:0] in2;
	input wire [p_nbits - 1:0] in3;
	input wire [p_nbits - 1:0] in4;
	input wire [p_nbits - 1:0] in5;
	input wire [p_nbits - 1:0] in6;
	input wire [p_nbits - 1:0] in7;
	input wire [2:0] sel;
	output reg [p_nbits - 1:0] out;
	always @(*)
		case (sel)
			3'd0: out = in0;
			3'd1: out = in1;
			3'd2: out = in2;
			3'd3: out = in3;
			3'd4: out = in4;
			3'd5: out = in5;
			3'd6: out = in6;
			3'd7: out = in7;
			default: out = {p_nbits {1'bx}};
		endcase
endmodule
module vc_MuxN (
	in,
	sel,
	out
);
	parameter p_nbits = 1;
	parameter p_ninputs = 2;
	input wire [(p_ninputs * p_nbits) - 1:0] in;
	input wire [$clog2(p_ninputs) - 1:0] sel;
	output wire [p_nbits - 1:0] out;
	assign out = in[sel * p_nbits+:p_nbits];
endmodule
module vc_Regfile_1r1w (
	clk,
	reset,
	read_addr,
	read_data,
	write_en,
	write_addr,
	write_data
);
	parameter p_data_nbits = 1;
	parameter p_num_entries = 2;
	parameter c_addr_nbits = $clog2(p_num_entries);
	input wire clk;
	input wire reset;
	input wire [c_addr_nbits - 1:0] read_addr;
	output wire [p_data_nbits - 1:0] read_data;
	input wire write_en;
	input wire [c_addr_nbits - 1:0] write_addr;
	input wire [p_data_nbits - 1:0] write_data;
	reg [p_data_nbits - 1:0] rfile [p_num_entries - 1:0];
	assign read_data = rfile[read_addr];
	always @(posedge clk)
		if (write_en)
			rfile[write_addr] <= write_data;
endmodule
module vc_ResetRegfile_1r1w (
	clk,
	reset,
	read_addr,
	read_data,
	write_en,
	write_addr,
	write_data
);
	parameter p_data_nbits = 1;
	parameter p_num_entries = 2;
	parameter p_reset_value = 0;
	parameter c_addr_nbits = $clog2(p_num_entries);
	input wire clk;
	input wire reset;
	input wire [c_addr_nbits - 1:0] read_addr;
	output wire [p_data_nbits - 1:0] read_data;
	input wire write_en;
	input wire [c_addr_nbits - 1:0] write_addr;
	input wire [p_data_nbits - 1:0] write_data;
	reg [p_data_nbits - 1:0] rfile [p_num_entries - 1:0];
	assign read_data = rfile[read_addr];
	genvar i;
	generate
		for (i = 0; i < p_num_entries; i = i + 1) begin : wport
			always @(posedge clk)
				if (reset)
					rfile[i] <= p_reset_value;
				else if (write_en && (i[c_addr_nbits - 1:0] == write_addr))
					rfile[i] <= write_data;
		end
	endgenerate
endmodule
module vc_Regfile_2r1w (
	clk,
	reset,
	read_addr0,
	read_data0,
	read_addr1,
	read_data1,
	write_en,
	write_addr,
	write_data
);
	parameter p_data_nbits = 1;
	parameter p_num_entries = 2;
	parameter c_addr_nbits = $clog2(p_num_entries);
	input wire clk;
	input wire reset;
	input wire [c_addr_nbits - 1:0] read_addr0;
	output wire [p_data_nbits - 1:0] read_data0;
	input wire [c_addr_nbits - 1:0] read_addr1;
	output wire [p_data_nbits - 1:0] read_data1;
	input wire write_en;
	input wire [c_addr_nbits - 1:0] write_addr;
	input wire [p_data_nbits - 1:0] write_data;
	reg [p_data_nbits - 1:0] rfile [p_num_entries - 1:0];
	assign read_data0 = rfile[read_addr0];
	assign read_data1 = rfile[read_addr1];
	always @(posedge clk)
		if (write_en)
			rfile[write_addr] <= write_data;
endmodule
module vc_Regfile_2r2w (
	clk,
	reset,
	read_addr0,
	read_data0,
	read_addr1,
	read_data1,
	write_en0,
	write_addr0,
	write_data0,
	write_en1,
	write_addr1,
	write_data1
);
	parameter p_data_nbits = 1;
	parameter p_num_entries = 2;
	parameter c_addr_nbits = $clog2(p_num_entries);
	input wire clk;
	input wire reset;
	input wire [c_addr_nbits - 1:0] read_addr0;
	output wire [p_data_nbits - 1:0] read_data0;
	input wire [c_addr_nbits - 1:0] read_addr1;
	output wire [p_data_nbits - 1:0] read_data1;
	input wire write_en0;
	input wire [c_addr_nbits - 1:0] write_addr0;
	input wire [p_data_nbits - 1:0] write_data0;
	input wire write_en1;
	input wire [c_addr_nbits - 1:0] write_addr1;
	input wire [p_data_nbits - 1:0] write_data1;
	reg [p_data_nbits - 1:0] rfile [p_num_entries - 1:0];
	assign read_data0 = rfile[read_addr0];
	assign read_data1 = rfile[read_addr1];
	always @(posedge clk) begin
		if (write_en0)
			rfile[write_addr0] <= write_data0;
		if (write_en1)
			rfile[write_addr1] <= write_data1;
	end
endmodule
module vc_Regfile_2r1w_zero (
	clk,
	reset,
	rd_addr0,
	rd_data0,
	rd_addr1,
	rd_data1,
	wr_en,
	wr_addr,
	wr_data
);
	input wire clk;
	input wire reset;
	input wire [4:0] rd_addr0;
	output wire [31:0] rd_data0;
	input wire [4:0] rd_addr1;
	output wire [31:0] rd_data1;
	input wire wr_en;
	input wire [4:0] wr_addr;
	input wire [31:0] wr_data;
	wire [31:0] rf_read_data0;
	wire [31:0] rf_read_data1;
	vc_Regfile_2r1w #(
		.p_data_nbits(32),
		.p_num_entries(32)
	) rfile(
		.clk(clk),
		.reset(reset),
		.read_addr0(rd_addr0),
		.read_data0(rf_read_data0),
		.read_addr1(rd_addr1),
		.read_data1(rf_read_data1),
		.write_en(wr_en),
		.write_addr(wr_addr),
		.write_data(wr_data)
	);
	assign rd_data0 = (rd_addr0 == 5'd0 ? 32'd0 : rf_read_data0);
	assign rd_data1 = (rd_addr1 == 5'd0 ? 32'd0 : rf_read_data1);
endmodule
module vc_Trace (
	clk,
	reset
);
	input wire clk;
	input wire reset;
	
endmodule
module vc_QueueCtrl1 (
	clk,
	reset,
	recv_val,
	recv_rdy,
	send_val,
	send_rdy,
	write_en,
	bypass_mux_sel,
	num_free_entries
);
	parameter p_type = 4'b0000;
	input wire clk;
	input wire reset;
	input wire recv_val;
	output wire recv_rdy;
	output wire send_val;
	input wire send_rdy;
	output wire write_en;
	output wire bypass_mux_sel;
	output wire num_free_entries;
	reg full;
	wire full_next;
	always @(posedge clk) full <= (reset ? 1'b0 : full_next);
	assign num_free_entries = (full ? 1'b0 : 1'b1);
	localparam c_pipe_en = |(p_type & 4'b0001);
	localparam c_bypass_en = |(p_type & 4'b0010);
	wire do_enq;
	assign do_enq = recv_rdy && recv_val;
	wire do_deq;
	assign do_deq = send_rdy && send_val;
	wire empty;
	assign empty = ~full;
	wire do_pipe;
	assign do_pipe = ((c_pipe_en && full) && do_enq) && do_deq;
	wire do_bypass;
	assign do_bypass = ((c_bypass_en && empty) && do_enq) && do_deq;
	assign write_en = do_enq && ~do_bypass;
	assign bypass_mux_sel = empty;
	assign recv_rdy = ~full || ((c_pipe_en && full) && send_rdy);
	assign send_val = ~empty || ((c_bypass_en && empty) && recv_val);
	assign full_next = (do_deq && ~do_pipe ? 1'b0 : (do_enq && ~do_bypass ? 1'b1 : full));
endmodule
module vc_QueueDpath1 (
	clk,
	reset,
	write_en,
	bypass_mux_sel,
	recv_msg,
	send_msg
);
	parameter p_type = 4'b0000;
	parameter p_msg_nbits = 1;
	input wire clk;
	input wire reset;
	input wire write_en;
	input wire bypass_mux_sel;
	input wire [p_msg_nbits - 1:0] recv_msg;
	output wire [p_msg_nbits - 1:0] send_msg;
	wire [p_msg_nbits - 1:0] qstore;
	vc_EnReg #(.p_nbits(p_msg_nbits)) qstore_reg(
		.clk(clk),
		.reset(reset),
		.en(write_en),
		.d(recv_msg),
		.q(qstore)
	);
	generate
		if (|(p_type & 4'b0010)) begin : genblk1
			vc_Mux2 #(.p_nbits(p_msg_nbits)) bypass_mux(
				.in0(qstore),
				.in1(recv_msg),
				.sel(bypass_mux_sel),
				.out(send_msg)
			);
		end
		else begin : genblk1
			assign send_msg = qstore;
		end
	endgenerate
endmodule
module vc_QueueCtrl (
	clk,
	reset,
	recv_val,
	recv_rdy,
	send_val,
	send_rdy,
	write_en,
	write_addr,
	read_addr,
	bypass_mux_sel,
	num_free_entries
);
	parameter p_type = 4'b0000;
	parameter p_num_msgs = 2;
	parameter c_addr_nbits = $clog2(p_num_msgs);
	input wire clk;
	input wire reset;
	input wire recv_val;
	output wire recv_rdy;
	output wire send_val;
	input wire send_rdy;
	output wire write_en;
	output wire [c_addr_nbits - 1:0] write_addr;
	output wire [c_addr_nbits - 1:0] read_addr;
	output wire bypass_mux_sel;
	output wire [c_addr_nbits:0] num_free_entries;
	wire [c_addr_nbits - 1:0] enq_ptr;
	wire [c_addr_nbits - 1:0] enq_ptr_next;
	vc_ResetReg #(.p_nbits(c_addr_nbits)) enq_ptr_reg(
		.clk(clk),
		.reset(reset),
		.d(enq_ptr_next),
		.q(enq_ptr)
	);
	wire [c_addr_nbits - 1:0] deq_ptr;
	wire [c_addr_nbits - 1:0] deq_ptr_next;
	vc_ResetReg #(.p_nbits(c_addr_nbits)) deq_ptr_reg(
		.clk(clk),
		.reset(reset),
		.d(deq_ptr_next),
		.q(deq_ptr)
	);
	assign write_addr = enq_ptr;
	assign read_addr = deq_ptr;
	wire full;
	wire full_next;
	vc_ResetReg #(.p_nbits(1)) full_reg(
		.clk(clk),
		.reset(reset),
		.d(full_next),
		.q(full)
	);
	localparam c_pipe_en = |(p_type & 4'b0001);
	localparam c_bypass_en = |(p_type & 4'b0010);
	wire do_enq;
	assign do_enq = recv_rdy && recv_val;
	wire do_deq;
	assign do_deq = send_rdy && send_val;
	wire empty;
	assign empty = ~full && (enq_ptr == deq_ptr);
	wire do_pipe;
	assign do_pipe = ((c_pipe_en && full) && do_enq) && do_deq;
	wire do_bypass;
	assign do_bypass = ((c_bypass_en && empty) && do_enq) && do_deq;
	assign write_en = do_enq && ~do_bypass;
	assign bypass_mux_sel = empty;
	assign recv_rdy = ~full || ((c_pipe_en && full) && send_rdy);
	assign send_val = ~empty || ((c_bypass_en && empty) && recv_val);
	wire [c_addr_nbits - 1:0] deq_ptr_plus1;
	assign deq_ptr_plus1 = deq_ptr + 1'b1;
	wire [c_addr_nbits - 1:0] deq_ptr_inc;
	assign deq_ptr_inc = (deq_ptr_plus1 == p_num_msgs ? {c_addr_nbits {1'b0}} : deq_ptr_plus1);
	wire [c_addr_nbits - 1:0] enq_ptr_plus1;
	assign enq_ptr_plus1 = enq_ptr + 1'b1;
	wire [c_addr_nbits - 1:0] enq_ptr_inc;
	assign enq_ptr_inc = (enq_ptr_plus1 == p_num_msgs ? {c_addr_nbits {1'b0}} : enq_ptr_plus1);
	assign deq_ptr_next = (do_deq && ~do_bypass ? deq_ptr_inc : deq_ptr);
	assign enq_ptr_next = (do_enq && ~do_bypass ? enq_ptr_inc : enq_ptr);
	assign full_next = ((do_enq && ~do_deq) && (enq_ptr_inc == deq_ptr) ? 1'b1 : ((do_deq && full) && ~do_pipe ? 1'b0 : full));
	assign num_free_entries = (full ? {c_addr_nbits + 1 {1'b0}} : (empty ? p_num_msgs[c_addr_nbits:0] : (enq_ptr > deq_ptr ? p_num_msgs[c_addr_nbits:0] - (enq_ptr - deq_ptr) : (deq_ptr > enq_ptr ? deq_ptr - enq_ptr : {c_addr_nbits + 1 {1'bx}}))));
endmodule
module vc_QueueDpath (
	clk,
	reset,
	write_en,
	bypass_mux_sel,
	write_addr,
	read_addr,
	recv_msg,
	send_msg
);
	parameter p_type = 4'b0000;
	parameter p_msg_nbits = 4;
	parameter p_num_msgs = 2;
	parameter c_addr_nbits = $clog2(p_num_msgs);
	input wire clk;
	input wire reset;
	input wire write_en;
	input wire bypass_mux_sel;
	input wire [c_addr_nbits - 1:0] write_addr;
	input wire [c_addr_nbits - 1:0] read_addr;
	input wire [p_msg_nbits - 1:0] recv_msg;
	output wire [p_msg_nbits - 1:0] send_msg;
	wire [p_msg_nbits - 1:0] read_data;
	vc_Regfile_1r1w #(
		.p_data_nbits(p_msg_nbits),
		.p_num_entries(p_num_msgs)
	) qstore(
		.clk(clk),
		.reset(reset),
		.read_addr(read_addr),
		.read_data(read_data),
		.write_en(write_en),
		.write_addr(write_addr),
		.write_data(recv_msg)
	);
	generate
		if (|(p_type & 4'b0010)) begin : genblk1
			vc_Mux2 #(.p_nbits(p_msg_nbits)) bypass_mux(
				.in0(read_data),
				.in1(recv_msg),
				.sel(bypass_mux_sel),
				.out(send_msg)
			);
		end
		else begin : genblk1
			assign send_msg = read_data;
		end
	endgenerate
endmodule
module vc_Queue (
	clk,
	reset,
	recv_val,
	recv_rdy,
	recv_msg,
	send_val,
	send_rdy,
	send_msg,
	num_free_entries
);
	parameter p_type = 4'b0000;
	parameter p_msg_nbits = 1;
	parameter p_num_msgs = 2;
	parameter c_addr_nbits = $clog2(p_num_msgs);
	input wire clk;
	input wire reset;
	input wire recv_val;
	output wire recv_rdy;
	input wire [p_msg_nbits - 1:0] recv_msg;
	output wire send_val;
	input wire send_rdy;
	output wire [p_msg_nbits - 1:0] send_msg;
	output wire [c_addr_nbits:0] num_free_entries;
	generate
		if (p_num_msgs == 1) begin : genblk1
			wire write_en;
			wire bypass_mux_sel;
			vc_QueueCtrl1 #(.p_type(p_type)) ctrl(
				.clk(clk),
				.reset(reset),
				.recv_val(recv_val),
				.recv_rdy(recv_rdy),
				.send_val(send_val),
				.send_rdy(send_rdy),
				.write_en(write_en),
				.bypass_mux_sel(bypass_mux_sel),
				.num_free_entries(num_free_entries)
			);
			vc_QueueDpath1 #(
				.p_type(p_type),
				.p_msg_nbits(p_msg_nbits)
			) dpath(
				.clk(clk),
				.reset(reset),
				.write_en(write_en),
				.bypass_mux_sel(bypass_mux_sel),
				.recv_msg(recv_msg),
				.send_msg(send_msg)
			);
		end
		else begin : genblk1
			wire write_en;
			wire bypass_mux_sel;
			wire [c_addr_nbits - 1:0] write_addr;
			wire [c_addr_nbits - 1:0] read_addr;
			vc_QueueCtrl #(
				.p_type(p_type),
				.p_num_msgs(p_num_msgs)
			) ctrl(
				.clk(clk),
				.reset(reset),
				.recv_val(recv_val),
				.recv_rdy(recv_rdy),
				.send_val(send_val),
				.send_rdy(send_rdy),
				.write_en(write_en),
				.write_addr(write_addr),
				.read_addr(read_addr),
				.bypass_mux_sel(bypass_mux_sel),
				.num_free_entries(num_free_entries)
			);
			vc_QueueDpath #(
				.p_type(p_type),
				.p_msg_nbits(p_msg_nbits),
				.p_num_msgs(p_num_msgs)
			) dpath(
				.clk(clk),
				.reset(reset),
				.write_en(write_en),
				.bypass_mux_sel(bypass_mux_sel),
				.write_addr(write_addr),
				.read_addr(read_addr),
				.recv_msg(recv_msg),
				.send_msg(send_msg)
			);
		end
	endgenerate
endmodule
module SPI_minion_components_SPIMinionAdapterVRTL (
	clk,
	reset,
	pull_en,
	pull_msg_val,
	pull_msg_spc,
	pull_msg_data,
	push_en,
	push_msg_val_wrt,
	push_msg_val_rd,
	push_msg_data,
	recv_msg,
	recv_rdy,
	recv_val,
	send_msg,
	send_rdy,
	send_val,
	parity
);
	parameter nbits = 8;
	parameter num_entries = 1;
	input wire clk;
	input wire reset;
	input wire pull_en;
	output reg pull_msg_val;
	output reg pull_msg_spc;
	output reg [nbits - 3:0] pull_msg_data;
	input wire push_en;
	input wire push_msg_val_wrt;
	input wire push_msg_val_rd;
	input wire [nbits - 3:0] push_msg_data;
	input wire [nbits - 3:0] recv_msg;
	output wire recv_rdy;
	input wire recv_val;
	output wire [nbits - 3:0] send_msg;
	input wire send_rdy;
	output wire send_val;
	output wire parity;
	reg open_entries;
	wire [nbits - 3:0] cm_q_send_msg;
	reg cm_q_send_rdy;
	wire cm_q_send_val;
	vc_Queue #(
		.p_type(4'b0000),
		.p_msg_nbits(nbits - 2),
		.p_num_msgs(num_entries)
	) cm_q(
		.clk(clk),
		.reset(reset),
		.recv_msg(recv_msg),
		.recv_rdy(recv_rdy),
		.recv_val(recv_val),
		.send_msg(cm_q_send_msg),
		.send_rdy(cm_q_send_rdy),
		.send_val(cm_q_send_val)
	);
	wire [$clog2(num_entries):0] mc_q_num_free;
	wire mc_q_recv_rdy;
	reg mc_q_recv_val;
	vc_Queue #(
		.p_type(4'b0000),
		.p_msg_nbits(nbits - 2),
		.p_num_msgs(num_entries)
	) mc_q(
		.clk(clk),
		.num_free_entries(mc_q_num_free),
		.reset(reset),
		.recv_msg(push_msg_data),
		.recv_rdy(mc_q_recv_rdy),
		.recv_val(mc_q_recv_val),
		.send_msg(send_msg),
		.send_rdy(send_rdy),
		.send_val(send_val)
	);
	assign parity = ^send_msg & send_val;
	always @(*) begin : comb_block
		open_entries = mc_q_num_free > 1;
		mc_q_recv_val = push_msg_val_wrt & push_en;
		pull_msg_spc = mc_q_recv_rdy & (~mc_q_recv_val | open_entries);
		cm_q_send_rdy = push_msg_val_rd & pull_en;
		pull_msg_val = cm_q_send_rdy & cm_q_send_val;
		pull_msg_data = cm_q_send_msg & {nbits - 2 {pull_msg_val}};
	end
endmodule
module SPIMinionAdapterConnectedVRTL (
	clk,
	reset,
	cs,
	sclk,
	mosi,
	miso,
	recv_msg,
	recv_rdy,
	recv_val,
	send_msg,
	send_rdy,
	send_val,
	minion_parity,
	adapter_parity
);
	parameter BIT_WIDTH = 32;
	parameter N_SAMPLES = 8;
	input wire clk;
	input wire reset;
	input wire cs;
	input wire sclk;
	input wire mosi;
	output wire miso;
	input wire [BIT_WIDTH - 1:0] recv_msg;
	output wire recv_rdy;
	input wire recv_val;
	output wire [BIT_WIDTH - 1:0] send_msg;
	input wire send_rdy;
	output wire send_val;
	output wire minion_parity;
	output wire adapter_parity;
	wire push_en;
	wire pull_en;
	wire [BIT_WIDTH + 1:0] push_msg;
	wire [BIT_WIDTH - 1:0] pull_msg;
	wire pull_msg_val;
	wire pull_msg_spc;
	SPI_minion_components_SPIMinionVRTL #(.nbits(BIT_WIDTH + 2)) minion(
		.clk(clk),
		.cs(cs),
		.miso(miso),
		.mosi(mosi),
		.reset(reset),
		.sclk(sclk),
		.pull_en(pull_en),
		.pull_msg({pull_msg_val, pull_msg_spc, pull_msg}),
		.push_en(push_en),
		.push_msg(push_msg),
		.parity(minion_parity)
	);
	SPI_minion_components_SPIMinionAdapterVRTL #(
		.nbits(BIT_WIDTH + 2),
		.num_entries(N_SAMPLES)
	) adapter1(
		.clk(clk),
		.reset(reset),
		.pull_en(pull_en),
		.pull_msg_val(pull_msg_val),
		.pull_msg_spc(pull_msg_spc),
		.pull_msg_data(pull_msg),
		.push_en(push_en),
		.push_msg_val_wrt(push_msg[BIT_WIDTH + 1]),
		.push_msg_val_rd(push_msg[BIT_WIDTH]),
		.push_msg_data(push_msg[BIT_WIDTH - 1:0]),
		.recv_msg(recv_msg),
		.recv_val(recv_val),
		.recv_rdy(recv_rdy),
		.send_msg(send_msg),
		.send_val(send_val),
		.send_rdy(send_rdy),
		.parity(adapter_parity)
	);
endmodule
module TwiddleGeneratorVRTL (
	sine_wave_in,
	twiddle_real,
	twiddle_imaginary
);
	parameter BIT_WIDTH = 4;
	parameter DECIMAL_PT = 2;
	parameter SIZE_FFT = 8;
	parameter STAGE_FFT = 0;
	input wire [(SIZE_FFT * BIT_WIDTH) - 1:0] sine_wave_in;
	output wire [((SIZE_FFT / 2) * BIT_WIDTH) - 1:0] twiddle_real;
	output wire [((SIZE_FFT / 2) * BIT_WIDTH) - 1:0] twiddle_imaginary;
	wire signed [31:0] trace;
	assign trace = 1'd1 << DECIMAL_PT;
	wire signed [31:0] trace2;
	assign trace2 = (1 * (SIZE_FFT / (2 * (2 ** STAGE_FFT)))) % SIZE_FFT;
	wire signed [31:0] trace3;
	assign trace3 = (2 * (SIZE_FFT / (2 * (2 ** STAGE_FFT)))) % SIZE_FFT;
	wire signed [31:0] trace4;
	assign trace4 = (3 * (SIZE_FFT / (2 * (2 ** STAGE_FFT)))) % SIZE_FFT;
	genvar m;
	generate
		for (m = 0; m < (2 ** STAGE_FFT); m = m + 1) begin : genblk1
			genvar i;
			for (i = 0; i < SIZE_FFT; i = i + (2 ** (STAGE_FFT + 1))) begin : genblk1
				if (m == 0) begin : genblk1
					assign twiddle_real[((i / 2) + m) * BIT_WIDTH+:BIT_WIDTH] = 1'b1 << DECIMAL_PT;
					assign twiddle_imaginary[((i / 2) + m) * BIT_WIDTH+:BIT_WIDTH] = 0;
				end
				else begin : genblk1
					assign twiddle_real[((i / 2) + m) * BIT_WIDTH+:BIT_WIDTH] = sine_wave_in[((SIZE_FFT - 1) - (((m * (SIZE_FFT / (2 * (2 ** STAGE_FFT)))) + (SIZE_FFT / 4)) % SIZE_FFT)) * BIT_WIDTH+:BIT_WIDTH];
					assign twiddle_imaginary[((i / 2) + m) * BIT_WIDTH+:BIT_WIDTH] = -sine_wave_in[((SIZE_FFT - 1) - ((m * (SIZE_FFT / (2 * (2 ** STAGE_FFT)))) % SIZE_FFT)) * BIT_WIDTH+:BIT_WIDTH];
				end
			end
		end
	endgenerate
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_512VRTL (sine_wave_out);
	output wire [16383:0] sine_wave_out;
	assign sine_wave_out[16352+:32] = 0;
	assign sine_wave_out[16320+:32] = 804;
	assign sine_wave_out[16288+:32] = 1608;
	assign sine_wave_out[16256+:32] = 2412;
	assign sine_wave_out[16224+:32] = 3215;
	assign sine_wave_out[16192+:32] = 4018;
	assign sine_wave_out[16160+:32] = 4821;
	assign sine_wave_out[16128+:32] = 5622;
	assign sine_wave_out[16096+:32] = 6423;
	assign sine_wave_out[16064+:32] = 7223;
	assign sine_wave_out[16032+:32] = 8022;
	assign sine_wave_out[16000+:32] = 8819;
	assign sine_wave_out[15968+:32] = 9616;
	assign sine_wave_out[15936+:32] = 10410;
	assign sine_wave_out[15904+:32] = 11204;
	assign sine_wave_out[15872+:32] = 11995;
	assign sine_wave_out[15840+:32] = 12785;
	assign sine_wave_out[15808+:32] = 13573;
	assign sine_wave_out[15776+:32] = 14359;
	assign sine_wave_out[15744+:32] = 15142;
	assign sine_wave_out[15712+:32] = 15923;
	assign sine_wave_out[15680+:32] = 16702;
	assign sine_wave_out[15648+:32] = 17479;
	assign sine_wave_out[15616+:32] = 18253;
	assign sine_wave_out[15584+:32] = 19024;
	assign sine_wave_out[15552+:32] = 19792;
	assign sine_wave_out[15520+:32] = 20557;
	assign sine_wave_out[15488+:32] = 21319;
	assign sine_wave_out[15456+:32] = 22078;
	assign sine_wave_out[15424+:32] = 22833;
	assign sine_wave_out[15392+:32] = 23586;
	assign sine_wave_out[15360+:32] = 24334;
	assign sine_wave_out[15328+:32] = 25079;
	assign sine_wave_out[15296+:32] = 25820;
	assign sine_wave_out[15264+:32] = 26557;
	assign sine_wave_out[15232+:32] = 27291;
	assign sine_wave_out[15200+:32] = 28020;
	assign sine_wave_out[15168+:32] = 28745;
	assign sine_wave_out[15136+:32] = 29465;
	assign sine_wave_out[15104+:32] = 30181;
	assign sine_wave_out[15072+:32] = 30893;
	assign sine_wave_out[15040+:32] = 31600;
	assign sine_wave_out[15008+:32] = 32302;
	assign sine_wave_out[14976+:32] = 32999;
	assign sine_wave_out[14944+:32] = 33692;
	assign sine_wave_out[14912+:32] = 34379;
	assign sine_wave_out[14880+:32] = 35061;
	assign sine_wave_out[14848+:32] = 35738;
	assign sine_wave_out[14816+:32] = 36409;
	assign sine_wave_out[14784+:32] = 37075;
	assign sine_wave_out[14752+:32] = 37736;
	assign sine_wave_out[14720+:32] = 38390;
	assign sine_wave_out[14688+:32] = 39039;
	assign sine_wave_out[14656+:32] = 39682;
	assign sine_wave_out[14624+:32] = 40319;
	assign sine_wave_out[14592+:32] = 40950;
	assign sine_wave_out[14560+:32] = 41575;
	assign sine_wave_out[14528+:32] = 42194;
	assign sine_wave_out[14496+:32] = 42806;
	assign sine_wave_out[14464+:32] = 43412;
	assign sine_wave_out[14432+:32] = 44011;
	assign sine_wave_out[14400+:32] = 44603;
	assign sine_wave_out[14368+:32] = 45189;
	assign sine_wave_out[14336+:32] = 45768;
	assign sine_wave_out[14304+:32] = 46340;
	assign sine_wave_out[14272+:32] = 46906;
	assign sine_wave_out[14240+:32] = 47464;
	assign sine_wave_out[14208+:32] = 48015;
	assign sine_wave_out[14176+:32] = 48558;
	assign sine_wave_out[14144+:32] = 49095;
	assign sine_wave_out[14112+:32] = 49624;
	assign sine_wave_out[14080+:32] = 50146;
	assign sine_wave_out[14048+:32] = 50660;
	assign sine_wave_out[14016+:32] = 51166;
	assign sine_wave_out[13984+:32] = 51665;
	assign sine_wave_out[13952+:32] = 52155;
	assign sine_wave_out[13920+:32] = 52639;
	assign sine_wave_out[13888+:32] = 53114;
	assign sine_wave_out[13856+:32] = 53581;
	assign sine_wave_out[13824+:32] = 54040;
	assign sine_wave_out[13792+:32] = 54491;
	assign sine_wave_out[13760+:32] = 54933;
	assign sine_wave_out[13728+:32] = 55368;
	assign sine_wave_out[13696+:32] = 55794;
	assign sine_wave_out[13664+:32] = 56212;
	assign sine_wave_out[13632+:32] = 56621;
	assign sine_wave_out[13600+:32] = 57022;
	assign sine_wave_out[13568+:32] = 57414;
	assign sine_wave_out[13536+:32] = 57797;
	assign sine_wave_out[13504+:32] = 58172;
	assign sine_wave_out[13472+:32] = 58538;
	assign sine_wave_out[13440+:32] = 58895;
	assign sine_wave_out[13408+:32] = 59243;
	assign sine_wave_out[13376+:32] = 59583;
	assign sine_wave_out[13344+:32] = 59913;
	assign sine_wave_out[13312+:32] = 60235;
	assign sine_wave_out[13280+:32] = 60547;
	assign sine_wave_out[13248+:32] = 60850;
	assign sine_wave_out[13216+:32] = 61144;
	assign sine_wave_out[13184+:32] = 61429;
	assign sine_wave_out[13152+:32] = 61705;
	assign sine_wave_out[13120+:32] = 61971;
	assign sine_wave_out[13088+:32] = 62228;
	assign sine_wave_out[13056+:32] = 62475;
	assign sine_wave_out[13024+:32] = 62714;
	assign sine_wave_out[12992+:32] = 62942;
	assign sine_wave_out[12960+:32] = 63162;
	assign sine_wave_out[12928+:32] = 63371;
	assign sine_wave_out[12896+:32] = 63571;
	assign sine_wave_out[12864+:32] = 63762;
	assign sine_wave_out[12832+:32] = 63943;
	assign sine_wave_out[12800+:32] = 64115;
	assign sine_wave_out[12768+:32] = 64276;
	assign sine_wave_out[12736+:32] = 64428;
	assign sine_wave_out[12704+:32] = 64571;
	assign sine_wave_out[12672+:32] = 64703;
	assign sine_wave_out[12640+:32] = 64826;
	assign sine_wave_out[12608+:32] = 64939;
	assign sine_wave_out[12576+:32] = 65043;
	assign sine_wave_out[12544+:32] = 65136;
	assign sine_wave_out[12512+:32] = 65220;
	assign sine_wave_out[12480+:32] = 65294;
	assign sine_wave_out[12448+:32] = 65358;
	assign sine_wave_out[12416+:32] = 65412;
	assign sine_wave_out[12384+:32] = 65457;
	assign sine_wave_out[12352+:32] = 65491;
	assign sine_wave_out[12320+:32] = 65516;
	assign sine_wave_out[12288+:32] = 65531;
	assign sine_wave_out[12256+:32] = 65536;
	assign sine_wave_out[12224+:32] = 65531;
	assign sine_wave_out[12192+:32] = 65516;
	assign sine_wave_out[12160+:32] = 65491;
	assign sine_wave_out[12128+:32] = 65457;
	assign sine_wave_out[12096+:32] = 65412;
	assign sine_wave_out[12064+:32] = 65358;
	assign sine_wave_out[12032+:32] = 65294;
	assign sine_wave_out[12000+:32] = 65220;
	assign sine_wave_out[11968+:32] = 65136;
	assign sine_wave_out[11936+:32] = 65043;
	assign sine_wave_out[11904+:32] = 64939;
	assign sine_wave_out[11872+:32] = 64826;
	assign sine_wave_out[11840+:32] = 64703;
	assign sine_wave_out[11808+:32] = 64571;
	assign sine_wave_out[11776+:32] = 64428;
	assign sine_wave_out[11744+:32] = 64276;
	assign sine_wave_out[11712+:32] = 64115;
	assign sine_wave_out[11680+:32] = 63943;
	assign sine_wave_out[11648+:32] = 63762;
	assign sine_wave_out[11616+:32] = 63571;
	assign sine_wave_out[11584+:32] = 63371;
	assign sine_wave_out[11552+:32] = 63162;
	assign sine_wave_out[11520+:32] = 62942;
	assign sine_wave_out[11488+:32] = 62714;
	assign sine_wave_out[11456+:32] = 62475;
	assign sine_wave_out[11424+:32] = 62228;
	assign sine_wave_out[11392+:32] = 61971;
	assign sine_wave_out[11360+:32] = 61705;
	assign sine_wave_out[11328+:32] = 61429;
	assign sine_wave_out[11296+:32] = 61144;
	assign sine_wave_out[11264+:32] = 60850;
	assign sine_wave_out[11232+:32] = 60547;
	assign sine_wave_out[11200+:32] = 60235;
	assign sine_wave_out[11168+:32] = 59913;
	assign sine_wave_out[11136+:32] = 59583;
	assign sine_wave_out[11104+:32] = 59243;
	assign sine_wave_out[11072+:32] = 58895;
	assign sine_wave_out[11040+:32] = 58538;
	assign sine_wave_out[11008+:32] = 58172;
	assign sine_wave_out[10976+:32] = 57797;
	assign sine_wave_out[10944+:32] = 57414;
	assign sine_wave_out[10912+:32] = 57022;
	assign sine_wave_out[10880+:32] = 56621;
	assign sine_wave_out[10848+:32] = 56212;
	assign sine_wave_out[10816+:32] = 55794;
	assign sine_wave_out[10784+:32] = 55368;
	assign sine_wave_out[10752+:32] = 54933;
	assign sine_wave_out[10720+:32] = 54491;
	assign sine_wave_out[10688+:32] = 54040;
	assign sine_wave_out[10656+:32] = 53581;
	assign sine_wave_out[10624+:32] = 53114;
	assign sine_wave_out[10592+:32] = 52639;
	assign sine_wave_out[10560+:32] = 52155;
	assign sine_wave_out[10528+:32] = 51665;
	assign sine_wave_out[10496+:32] = 51166;
	assign sine_wave_out[10464+:32] = 50660;
	assign sine_wave_out[10432+:32] = 50146;
	assign sine_wave_out[10400+:32] = 49624;
	assign sine_wave_out[10368+:32] = 49095;
	assign sine_wave_out[10336+:32] = 48558;
	assign sine_wave_out[10304+:32] = 48015;
	assign sine_wave_out[10272+:32] = 47464;
	assign sine_wave_out[10240+:32] = 46906;
	assign sine_wave_out[10208+:32] = 46340;
	assign sine_wave_out[10176+:32] = 45768;
	assign sine_wave_out[10144+:32] = 45189;
	assign sine_wave_out[10112+:32] = 44603;
	assign sine_wave_out[10080+:32] = 44011;
	assign sine_wave_out[10048+:32] = 43412;
	assign sine_wave_out[10016+:32] = 42806;
	assign sine_wave_out[9984+:32] = 42194;
	assign sine_wave_out[9952+:32] = 41575;
	assign sine_wave_out[9920+:32] = 40950;
	assign sine_wave_out[9888+:32] = 40319;
	assign sine_wave_out[9856+:32] = 39682;
	assign sine_wave_out[9824+:32] = 39039;
	assign sine_wave_out[9792+:32] = 38390;
	assign sine_wave_out[9760+:32] = 37736;
	assign sine_wave_out[9728+:32] = 37075;
	assign sine_wave_out[9696+:32] = 36409;
	assign sine_wave_out[9664+:32] = 35738;
	assign sine_wave_out[9632+:32] = 35061;
	assign sine_wave_out[9600+:32] = 34379;
	assign sine_wave_out[9568+:32] = 33692;
	assign sine_wave_out[9536+:32] = 32999;
	assign sine_wave_out[9504+:32] = 32302;
	assign sine_wave_out[9472+:32] = 31600;
	assign sine_wave_out[9440+:32] = 30893;
	assign sine_wave_out[9408+:32] = 30181;
	assign sine_wave_out[9376+:32] = 29465;
	assign sine_wave_out[9344+:32] = 28745;
	assign sine_wave_out[9312+:32] = 28020;
	assign sine_wave_out[9280+:32] = 27291;
	assign sine_wave_out[9248+:32] = 26557;
	assign sine_wave_out[9216+:32] = 25820;
	assign sine_wave_out[9184+:32] = 25079;
	assign sine_wave_out[9152+:32] = 24334;
	assign sine_wave_out[9120+:32] = 23586;
	assign sine_wave_out[9088+:32] = 22833;
	assign sine_wave_out[9056+:32] = 22078;
	assign sine_wave_out[9024+:32] = 21319;
	assign sine_wave_out[8992+:32] = 20557;
	assign sine_wave_out[8960+:32] = 19792;
	assign sine_wave_out[8928+:32] = 19024;
	assign sine_wave_out[8896+:32] = 18253;
	assign sine_wave_out[8864+:32] = 17479;
	assign sine_wave_out[8832+:32] = 16702;
	assign sine_wave_out[8800+:32] = 15923;
	assign sine_wave_out[8768+:32] = 15142;
	assign sine_wave_out[8736+:32] = 14359;
	assign sine_wave_out[8704+:32] = 13573;
	assign sine_wave_out[8672+:32] = 12785;
	assign sine_wave_out[8640+:32] = 11995;
	assign sine_wave_out[8608+:32] = 11204;
	assign sine_wave_out[8576+:32] = 10410;
	assign sine_wave_out[8544+:32] = 9616;
	assign sine_wave_out[8512+:32] = 8819;
	assign sine_wave_out[8480+:32] = 8022;
	assign sine_wave_out[8448+:32] = 7223;
	assign sine_wave_out[8416+:32] = 6423;
	assign sine_wave_out[8384+:32] = 5622;
	assign sine_wave_out[8352+:32] = 4821;
	assign sine_wave_out[8320+:32] = 4018;
	assign sine_wave_out[8288+:32] = 3215;
	assign sine_wave_out[8256+:32] = 2412;
	assign sine_wave_out[8224+:32] = 1608;
	assign sine_wave_out[8192+:32] = 804;
	assign sine_wave_out[8160+:32] = 0;
	assign sine_wave_out[8128+:32] = -804;
	assign sine_wave_out[8096+:32] = -1608;
	assign sine_wave_out[8064+:32] = -2412;
	assign sine_wave_out[8032+:32] = -3215;
	assign sine_wave_out[8000+:32] = -4018;
	assign sine_wave_out[7968+:32] = -4821;
	assign sine_wave_out[7936+:32] = -5622;
	assign sine_wave_out[7904+:32] = -6423;
	assign sine_wave_out[7872+:32] = -7223;
	assign sine_wave_out[7840+:32] = -8022;
	assign sine_wave_out[7808+:32] = -8819;
	assign sine_wave_out[7776+:32] = -9616;
	assign sine_wave_out[7744+:32] = -10410;
	assign sine_wave_out[7712+:32] = -11204;
	assign sine_wave_out[7680+:32] = -11995;
	assign sine_wave_out[7648+:32] = -12785;
	assign sine_wave_out[7616+:32] = -13573;
	assign sine_wave_out[7584+:32] = -14359;
	assign sine_wave_out[7552+:32] = -15142;
	assign sine_wave_out[7520+:32] = -15923;
	assign sine_wave_out[7488+:32] = -16702;
	assign sine_wave_out[7456+:32] = -17479;
	assign sine_wave_out[7424+:32] = -18253;
	assign sine_wave_out[7392+:32] = -19024;
	assign sine_wave_out[7360+:32] = -19792;
	assign sine_wave_out[7328+:32] = -20557;
	assign sine_wave_out[7296+:32] = -21319;
	assign sine_wave_out[7264+:32] = -22078;
	assign sine_wave_out[7232+:32] = -22833;
	assign sine_wave_out[7200+:32] = -23586;
	assign sine_wave_out[7168+:32] = -24334;
	assign sine_wave_out[7136+:32] = -25079;
	assign sine_wave_out[7104+:32] = -25820;
	assign sine_wave_out[7072+:32] = -26557;
	assign sine_wave_out[7040+:32] = -27291;
	assign sine_wave_out[7008+:32] = -28020;
	assign sine_wave_out[6976+:32] = -28745;
	assign sine_wave_out[6944+:32] = -29465;
	assign sine_wave_out[6912+:32] = -30181;
	assign sine_wave_out[6880+:32] = -30893;
	assign sine_wave_out[6848+:32] = -31600;
	assign sine_wave_out[6816+:32] = -32302;
	assign sine_wave_out[6784+:32] = -32999;
	assign sine_wave_out[6752+:32] = -33692;
	assign sine_wave_out[6720+:32] = -34379;
	assign sine_wave_out[6688+:32] = -35061;
	assign sine_wave_out[6656+:32] = -35738;
	assign sine_wave_out[6624+:32] = -36409;
	assign sine_wave_out[6592+:32] = -37075;
	assign sine_wave_out[6560+:32] = -37736;
	assign sine_wave_out[6528+:32] = -38390;
	assign sine_wave_out[6496+:32] = -39039;
	assign sine_wave_out[6464+:32] = -39682;
	assign sine_wave_out[6432+:32] = -40319;
	assign sine_wave_out[6400+:32] = -40950;
	assign sine_wave_out[6368+:32] = -41575;
	assign sine_wave_out[6336+:32] = -42194;
	assign sine_wave_out[6304+:32] = -42806;
	assign sine_wave_out[6272+:32] = -43412;
	assign sine_wave_out[6240+:32] = -44011;
	assign sine_wave_out[6208+:32] = -44603;
	assign sine_wave_out[6176+:32] = -45189;
	assign sine_wave_out[6144+:32] = -45768;
	assign sine_wave_out[6112+:32] = -46340;
	assign sine_wave_out[6080+:32] = -46906;
	assign sine_wave_out[6048+:32] = -47464;
	assign sine_wave_out[6016+:32] = -48015;
	assign sine_wave_out[5984+:32] = -48558;
	assign sine_wave_out[5952+:32] = -49095;
	assign sine_wave_out[5920+:32] = -49624;
	assign sine_wave_out[5888+:32] = -50146;
	assign sine_wave_out[5856+:32] = -50660;
	assign sine_wave_out[5824+:32] = -51166;
	assign sine_wave_out[5792+:32] = -51665;
	assign sine_wave_out[5760+:32] = -52155;
	assign sine_wave_out[5728+:32] = -52639;
	assign sine_wave_out[5696+:32] = -53114;
	assign sine_wave_out[5664+:32] = -53581;
	assign sine_wave_out[5632+:32] = -54040;
	assign sine_wave_out[5600+:32] = -54491;
	assign sine_wave_out[5568+:32] = -54933;
	assign sine_wave_out[5536+:32] = -55368;
	assign sine_wave_out[5504+:32] = -55794;
	assign sine_wave_out[5472+:32] = -56212;
	assign sine_wave_out[5440+:32] = -56621;
	assign sine_wave_out[5408+:32] = -57022;
	assign sine_wave_out[5376+:32] = -57414;
	assign sine_wave_out[5344+:32] = -57797;
	assign sine_wave_out[5312+:32] = -58172;
	assign sine_wave_out[5280+:32] = -58538;
	assign sine_wave_out[5248+:32] = -58895;
	assign sine_wave_out[5216+:32] = -59243;
	assign sine_wave_out[5184+:32] = -59583;
	assign sine_wave_out[5152+:32] = -59913;
	assign sine_wave_out[5120+:32] = -60235;
	assign sine_wave_out[5088+:32] = -60547;
	assign sine_wave_out[5056+:32] = -60850;
	assign sine_wave_out[5024+:32] = -61144;
	assign sine_wave_out[4992+:32] = -61429;
	assign sine_wave_out[4960+:32] = -61705;
	assign sine_wave_out[4928+:32] = -61971;
	assign sine_wave_out[4896+:32] = -62228;
	assign sine_wave_out[4864+:32] = -62475;
	assign sine_wave_out[4832+:32] = -62714;
	assign sine_wave_out[4800+:32] = -62942;
	assign sine_wave_out[4768+:32] = -63162;
	assign sine_wave_out[4736+:32] = -63371;
	assign sine_wave_out[4704+:32] = -63571;
	assign sine_wave_out[4672+:32] = -63762;
	assign sine_wave_out[4640+:32] = -63943;
	assign sine_wave_out[4608+:32] = -64115;
	assign sine_wave_out[4576+:32] = -64276;
	assign sine_wave_out[4544+:32] = -64428;
	assign sine_wave_out[4512+:32] = -64571;
	assign sine_wave_out[4480+:32] = -64703;
	assign sine_wave_out[4448+:32] = -64826;
	assign sine_wave_out[4416+:32] = -64939;
	assign sine_wave_out[4384+:32] = -65043;
	assign sine_wave_out[4352+:32] = -65136;
	assign sine_wave_out[4320+:32] = -65220;
	assign sine_wave_out[4288+:32] = -65294;
	assign sine_wave_out[4256+:32] = -65358;
	assign sine_wave_out[4224+:32] = -65412;
	assign sine_wave_out[4192+:32] = -65457;
	assign sine_wave_out[4160+:32] = -65491;
	assign sine_wave_out[4128+:32] = -65516;
	assign sine_wave_out[4096+:32] = -65531;
	assign sine_wave_out[4064+:32] = -65536;
	assign sine_wave_out[4032+:32] = -65531;
	assign sine_wave_out[4000+:32] = -65516;
	assign sine_wave_out[3968+:32] = -65491;
	assign sine_wave_out[3936+:32] = -65457;
	assign sine_wave_out[3904+:32] = -65412;
	assign sine_wave_out[3872+:32] = -65358;
	assign sine_wave_out[3840+:32] = -65294;
	assign sine_wave_out[3808+:32] = -65220;
	assign sine_wave_out[3776+:32] = -65136;
	assign sine_wave_out[3744+:32] = -65043;
	assign sine_wave_out[3712+:32] = -64939;
	assign sine_wave_out[3680+:32] = -64826;
	assign sine_wave_out[3648+:32] = -64703;
	assign sine_wave_out[3616+:32] = -64571;
	assign sine_wave_out[3584+:32] = -64428;
	assign sine_wave_out[3552+:32] = -64276;
	assign sine_wave_out[3520+:32] = -64115;
	assign sine_wave_out[3488+:32] = -63943;
	assign sine_wave_out[3456+:32] = -63762;
	assign sine_wave_out[3424+:32] = -63571;
	assign sine_wave_out[3392+:32] = -63371;
	assign sine_wave_out[3360+:32] = -63162;
	assign sine_wave_out[3328+:32] = -62942;
	assign sine_wave_out[3296+:32] = -62714;
	assign sine_wave_out[3264+:32] = -62475;
	assign sine_wave_out[3232+:32] = -62228;
	assign sine_wave_out[3200+:32] = -61971;
	assign sine_wave_out[3168+:32] = -61705;
	assign sine_wave_out[3136+:32] = -61429;
	assign sine_wave_out[3104+:32] = -61144;
	assign sine_wave_out[3072+:32] = -60850;
	assign sine_wave_out[3040+:32] = -60547;
	assign sine_wave_out[3008+:32] = -60235;
	assign sine_wave_out[2976+:32] = -59913;
	assign sine_wave_out[2944+:32] = -59583;
	assign sine_wave_out[2912+:32] = -59243;
	assign sine_wave_out[2880+:32] = -58895;
	assign sine_wave_out[2848+:32] = -58538;
	assign sine_wave_out[2816+:32] = -58172;
	assign sine_wave_out[2784+:32] = -57797;
	assign sine_wave_out[2752+:32] = -57414;
	assign sine_wave_out[2720+:32] = -57022;
	assign sine_wave_out[2688+:32] = -56621;
	assign sine_wave_out[2656+:32] = -56212;
	assign sine_wave_out[2624+:32] = -55794;
	assign sine_wave_out[2592+:32] = -55368;
	assign sine_wave_out[2560+:32] = -54933;
	assign sine_wave_out[2528+:32] = -54491;
	assign sine_wave_out[2496+:32] = -54040;
	assign sine_wave_out[2464+:32] = -53581;
	assign sine_wave_out[2432+:32] = -53114;
	assign sine_wave_out[2400+:32] = -52639;
	assign sine_wave_out[2368+:32] = -52155;
	assign sine_wave_out[2336+:32] = -51665;
	assign sine_wave_out[2304+:32] = -51166;
	assign sine_wave_out[2272+:32] = -50660;
	assign sine_wave_out[2240+:32] = -50146;
	assign sine_wave_out[2208+:32] = -49624;
	assign sine_wave_out[2176+:32] = -49095;
	assign sine_wave_out[2144+:32] = -48558;
	assign sine_wave_out[2112+:32] = -48015;
	assign sine_wave_out[2080+:32] = -47464;
	assign sine_wave_out[2048+:32] = -46906;
	assign sine_wave_out[2016+:32] = -46340;
	assign sine_wave_out[1984+:32] = -45768;
	assign sine_wave_out[1952+:32] = -45189;
	assign sine_wave_out[1920+:32] = -44603;
	assign sine_wave_out[1888+:32] = -44011;
	assign sine_wave_out[1856+:32] = -43412;
	assign sine_wave_out[1824+:32] = -42806;
	assign sine_wave_out[1792+:32] = -42194;
	assign sine_wave_out[1760+:32] = -41575;
	assign sine_wave_out[1728+:32] = -40950;
	assign sine_wave_out[1696+:32] = -40319;
	assign sine_wave_out[1664+:32] = -39682;
	assign sine_wave_out[1632+:32] = -39039;
	assign sine_wave_out[1600+:32] = -38390;
	assign sine_wave_out[1568+:32] = -37736;
	assign sine_wave_out[1536+:32] = -37075;
	assign sine_wave_out[1504+:32] = -36409;
	assign sine_wave_out[1472+:32] = -35738;
	assign sine_wave_out[1440+:32] = -35061;
	assign sine_wave_out[1408+:32] = -34379;
	assign sine_wave_out[1376+:32] = -33692;
	assign sine_wave_out[1344+:32] = -32999;
	assign sine_wave_out[1312+:32] = -32302;
	assign sine_wave_out[1280+:32] = -31600;
	assign sine_wave_out[1248+:32] = -30893;
	assign sine_wave_out[1216+:32] = -30181;
	assign sine_wave_out[1184+:32] = -29465;
	assign sine_wave_out[1152+:32] = -28745;
	assign sine_wave_out[1120+:32] = -28020;
	assign sine_wave_out[1088+:32] = -27291;
	assign sine_wave_out[1056+:32] = -26557;
	assign sine_wave_out[1024+:32] = -25820;
	assign sine_wave_out[992+:32] = -25079;
	assign sine_wave_out[960+:32] = -24334;
	assign sine_wave_out[928+:32] = -23586;
	assign sine_wave_out[896+:32] = -22833;
	assign sine_wave_out[864+:32] = -22078;
	assign sine_wave_out[832+:32] = -21319;
	assign sine_wave_out[800+:32] = -20557;
	assign sine_wave_out[768+:32] = -19792;
	assign sine_wave_out[736+:32] = -19024;
	assign sine_wave_out[704+:32] = -18253;
	assign sine_wave_out[672+:32] = -17479;
	assign sine_wave_out[640+:32] = -16702;
	assign sine_wave_out[608+:32] = -15923;
	assign sine_wave_out[576+:32] = -15142;
	assign sine_wave_out[544+:32] = -14359;
	assign sine_wave_out[512+:32] = -13573;
	assign sine_wave_out[480+:32] = -12785;
	assign sine_wave_out[448+:32] = -11995;
	assign sine_wave_out[416+:32] = -11204;
	assign sine_wave_out[384+:32] = -10410;
	assign sine_wave_out[352+:32] = -9616;
	assign sine_wave_out[320+:32] = -8819;
	assign sine_wave_out[288+:32] = -8022;
	assign sine_wave_out[256+:32] = -7223;
	assign sine_wave_out[224+:32] = -6423;
	assign sine_wave_out[192+:32] = -5622;
	assign sine_wave_out[160+:32] = -4821;
	assign sine_wave_out[128+:32] = -4018;
	assign sine_wave_out[96+:32] = -3215;
	assign sine_wave_out[64+:32] = -2412;
	assign sine_wave_out[32+:32] = -1608;
	assign sine_wave_out[0+:32] = -804;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_256VRTL (sine_wave_out);
	output wire [8191:0] sine_wave_out;
	assign sine_wave_out[8160+:32] = 0;
	assign sine_wave_out[8128+:32] = 1608;
	assign sine_wave_out[8096+:32] = 3215;
	assign sine_wave_out[8064+:32] = 4821;
	assign sine_wave_out[8032+:32] = 6423;
	assign sine_wave_out[8000+:32] = 8022;
	assign sine_wave_out[7968+:32] = 9616;
	assign sine_wave_out[7936+:32] = 11204;
	assign sine_wave_out[7904+:32] = 12785;
	assign sine_wave_out[7872+:32] = 14359;
	assign sine_wave_out[7840+:32] = 15923;
	assign sine_wave_out[7808+:32] = 17479;
	assign sine_wave_out[7776+:32] = 19024;
	assign sine_wave_out[7744+:32] = 20557;
	assign sine_wave_out[7712+:32] = 22078;
	assign sine_wave_out[7680+:32] = 23586;
	assign sine_wave_out[7648+:32] = 25079;
	assign sine_wave_out[7616+:32] = 26557;
	assign sine_wave_out[7584+:32] = 28020;
	assign sine_wave_out[7552+:32] = 29465;
	assign sine_wave_out[7520+:32] = 30893;
	assign sine_wave_out[7488+:32] = 32302;
	assign sine_wave_out[7456+:32] = 33692;
	assign sine_wave_out[7424+:32] = 35061;
	assign sine_wave_out[7392+:32] = 36409;
	assign sine_wave_out[7360+:32] = 37736;
	assign sine_wave_out[7328+:32] = 39039;
	assign sine_wave_out[7296+:32] = 40319;
	assign sine_wave_out[7264+:32] = 41575;
	assign sine_wave_out[7232+:32] = 42806;
	assign sine_wave_out[7200+:32] = 44011;
	assign sine_wave_out[7168+:32] = 45189;
	assign sine_wave_out[7136+:32] = 46340;
	assign sine_wave_out[7104+:32] = 47464;
	assign sine_wave_out[7072+:32] = 48558;
	assign sine_wave_out[7040+:32] = 49624;
	assign sine_wave_out[7008+:32] = 50660;
	assign sine_wave_out[6976+:32] = 51665;
	assign sine_wave_out[6944+:32] = 52639;
	assign sine_wave_out[6912+:32] = 53581;
	assign sine_wave_out[6880+:32] = 54491;
	assign sine_wave_out[6848+:32] = 55368;
	assign sine_wave_out[6816+:32] = 56212;
	assign sine_wave_out[6784+:32] = 57022;
	assign sine_wave_out[6752+:32] = 57797;
	assign sine_wave_out[6720+:32] = 58538;
	assign sine_wave_out[6688+:32] = 59243;
	assign sine_wave_out[6656+:32] = 59913;
	assign sine_wave_out[6624+:32] = 60547;
	assign sine_wave_out[6592+:32] = 61144;
	assign sine_wave_out[6560+:32] = 61705;
	assign sine_wave_out[6528+:32] = 62228;
	assign sine_wave_out[6496+:32] = 62714;
	assign sine_wave_out[6464+:32] = 63162;
	assign sine_wave_out[6432+:32] = 63571;
	assign sine_wave_out[6400+:32] = 63943;
	assign sine_wave_out[6368+:32] = 64276;
	assign sine_wave_out[6336+:32] = 64571;
	assign sine_wave_out[6304+:32] = 64826;
	assign sine_wave_out[6272+:32] = 65043;
	assign sine_wave_out[6240+:32] = 65220;
	assign sine_wave_out[6208+:32] = 65358;
	assign sine_wave_out[6176+:32] = 65457;
	assign sine_wave_out[6144+:32] = 65516;
	assign sine_wave_out[6112+:32] = 65536;
	assign sine_wave_out[6080+:32] = 65516;
	assign sine_wave_out[6048+:32] = 65457;
	assign sine_wave_out[6016+:32] = 65358;
	assign sine_wave_out[5984+:32] = 65220;
	assign sine_wave_out[5952+:32] = 65043;
	assign sine_wave_out[5920+:32] = 64826;
	assign sine_wave_out[5888+:32] = 64571;
	assign sine_wave_out[5856+:32] = 64276;
	assign sine_wave_out[5824+:32] = 63943;
	assign sine_wave_out[5792+:32] = 63571;
	assign sine_wave_out[5760+:32] = 63162;
	assign sine_wave_out[5728+:32] = 62714;
	assign sine_wave_out[5696+:32] = 62228;
	assign sine_wave_out[5664+:32] = 61705;
	assign sine_wave_out[5632+:32] = 61144;
	assign sine_wave_out[5600+:32] = 60547;
	assign sine_wave_out[5568+:32] = 59913;
	assign sine_wave_out[5536+:32] = 59243;
	assign sine_wave_out[5504+:32] = 58538;
	assign sine_wave_out[5472+:32] = 57797;
	assign sine_wave_out[5440+:32] = 57022;
	assign sine_wave_out[5408+:32] = 56212;
	assign sine_wave_out[5376+:32] = 55368;
	assign sine_wave_out[5344+:32] = 54491;
	assign sine_wave_out[5312+:32] = 53581;
	assign sine_wave_out[5280+:32] = 52639;
	assign sine_wave_out[5248+:32] = 51665;
	assign sine_wave_out[5216+:32] = 50660;
	assign sine_wave_out[5184+:32] = 49624;
	assign sine_wave_out[5152+:32] = 48558;
	assign sine_wave_out[5120+:32] = 47464;
	assign sine_wave_out[5088+:32] = 46340;
	assign sine_wave_out[5056+:32] = 45189;
	assign sine_wave_out[5024+:32] = 44011;
	assign sine_wave_out[4992+:32] = 42806;
	assign sine_wave_out[4960+:32] = 41575;
	assign sine_wave_out[4928+:32] = 40319;
	assign sine_wave_out[4896+:32] = 39039;
	assign sine_wave_out[4864+:32] = 37736;
	assign sine_wave_out[4832+:32] = 36409;
	assign sine_wave_out[4800+:32] = 35061;
	assign sine_wave_out[4768+:32] = 33692;
	assign sine_wave_out[4736+:32] = 32302;
	assign sine_wave_out[4704+:32] = 30893;
	assign sine_wave_out[4672+:32] = 29465;
	assign sine_wave_out[4640+:32] = 28020;
	assign sine_wave_out[4608+:32] = 26557;
	assign sine_wave_out[4576+:32] = 25079;
	assign sine_wave_out[4544+:32] = 23586;
	assign sine_wave_out[4512+:32] = 22078;
	assign sine_wave_out[4480+:32] = 20557;
	assign sine_wave_out[4448+:32] = 19024;
	assign sine_wave_out[4416+:32] = 17479;
	assign sine_wave_out[4384+:32] = 15923;
	assign sine_wave_out[4352+:32] = 14359;
	assign sine_wave_out[4320+:32] = 12785;
	assign sine_wave_out[4288+:32] = 11204;
	assign sine_wave_out[4256+:32] = 9616;
	assign sine_wave_out[4224+:32] = 8022;
	assign sine_wave_out[4192+:32] = 6423;
	assign sine_wave_out[4160+:32] = 4821;
	assign sine_wave_out[4128+:32] = 3215;
	assign sine_wave_out[4096+:32] = 1608;
	assign sine_wave_out[4064+:32] = 0;
	assign sine_wave_out[4032+:32] = -1608;
	assign sine_wave_out[4000+:32] = -3215;
	assign sine_wave_out[3968+:32] = -4821;
	assign sine_wave_out[3936+:32] = -6423;
	assign sine_wave_out[3904+:32] = -8022;
	assign sine_wave_out[3872+:32] = -9616;
	assign sine_wave_out[3840+:32] = -11204;
	assign sine_wave_out[3808+:32] = -12785;
	assign sine_wave_out[3776+:32] = -14359;
	assign sine_wave_out[3744+:32] = -15923;
	assign sine_wave_out[3712+:32] = -17479;
	assign sine_wave_out[3680+:32] = -19024;
	assign sine_wave_out[3648+:32] = -20557;
	assign sine_wave_out[3616+:32] = -22078;
	assign sine_wave_out[3584+:32] = -23586;
	assign sine_wave_out[3552+:32] = -25079;
	assign sine_wave_out[3520+:32] = -26557;
	assign sine_wave_out[3488+:32] = -28020;
	assign sine_wave_out[3456+:32] = -29465;
	assign sine_wave_out[3424+:32] = -30893;
	assign sine_wave_out[3392+:32] = -32302;
	assign sine_wave_out[3360+:32] = -33692;
	assign sine_wave_out[3328+:32] = -35061;
	assign sine_wave_out[3296+:32] = -36409;
	assign sine_wave_out[3264+:32] = -37736;
	assign sine_wave_out[3232+:32] = -39039;
	assign sine_wave_out[3200+:32] = -40319;
	assign sine_wave_out[3168+:32] = -41575;
	assign sine_wave_out[3136+:32] = -42806;
	assign sine_wave_out[3104+:32] = -44011;
	assign sine_wave_out[3072+:32] = -45189;
	assign sine_wave_out[3040+:32] = -46340;
	assign sine_wave_out[3008+:32] = -47464;
	assign sine_wave_out[2976+:32] = -48558;
	assign sine_wave_out[2944+:32] = -49624;
	assign sine_wave_out[2912+:32] = -50660;
	assign sine_wave_out[2880+:32] = -51665;
	assign sine_wave_out[2848+:32] = -52639;
	assign sine_wave_out[2816+:32] = -53581;
	assign sine_wave_out[2784+:32] = -54491;
	assign sine_wave_out[2752+:32] = -55368;
	assign sine_wave_out[2720+:32] = -56212;
	assign sine_wave_out[2688+:32] = -57022;
	assign sine_wave_out[2656+:32] = -57797;
	assign sine_wave_out[2624+:32] = -58538;
	assign sine_wave_out[2592+:32] = -59243;
	assign sine_wave_out[2560+:32] = -59913;
	assign sine_wave_out[2528+:32] = -60547;
	assign sine_wave_out[2496+:32] = -61144;
	assign sine_wave_out[2464+:32] = -61705;
	assign sine_wave_out[2432+:32] = -62228;
	assign sine_wave_out[2400+:32] = -62714;
	assign sine_wave_out[2368+:32] = -63162;
	assign sine_wave_out[2336+:32] = -63571;
	assign sine_wave_out[2304+:32] = -63943;
	assign sine_wave_out[2272+:32] = -64276;
	assign sine_wave_out[2240+:32] = -64571;
	assign sine_wave_out[2208+:32] = -64826;
	assign sine_wave_out[2176+:32] = -65043;
	assign sine_wave_out[2144+:32] = -65220;
	assign sine_wave_out[2112+:32] = -65358;
	assign sine_wave_out[2080+:32] = -65457;
	assign sine_wave_out[2048+:32] = -65516;
	assign sine_wave_out[2016+:32] = -65536;
	assign sine_wave_out[1984+:32] = -65516;
	assign sine_wave_out[1952+:32] = -65457;
	assign sine_wave_out[1920+:32] = -65358;
	assign sine_wave_out[1888+:32] = -65220;
	assign sine_wave_out[1856+:32] = -65043;
	assign sine_wave_out[1824+:32] = -64826;
	assign sine_wave_out[1792+:32] = -64571;
	assign sine_wave_out[1760+:32] = -64276;
	assign sine_wave_out[1728+:32] = -63943;
	assign sine_wave_out[1696+:32] = -63571;
	assign sine_wave_out[1664+:32] = -63162;
	assign sine_wave_out[1632+:32] = -62714;
	assign sine_wave_out[1600+:32] = -62228;
	assign sine_wave_out[1568+:32] = -61705;
	assign sine_wave_out[1536+:32] = -61144;
	assign sine_wave_out[1504+:32] = -60547;
	assign sine_wave_out[1472+:32] = -59913;
	assign sine_wave_out[1440+:32] = -59243;
	assign sine_wave_out[1408+:32] = -58538;
	assign sine_wave_out[1376+:32] = -57797;
	assign sine_wave_out[1344+:32] = -57022;
	assign sine_wave_out[1312+:32] = -56212;
	assign sine_wave_out[1280+:32] = -55368;
	assign sine_wave_out[1248+:32] = -54491;
	assign sine_wave_out[1216+:32] = -53581;
	assign sine_wave_out[1184+:32] = -52639;
	assign sine_wave_out[1152+:32] = -51665;
	assign sine_wave_out[1120+:32] = -50660;
	assign sine_wave_out[1088+:32] = -49624;
	assign sine_wave_out[1056+:32] = -48558;
	assign sine_wave_out[1024+:32] = -47464;
	assign sine_wave_out[992+:32] = -46340;
	assign sine_wave_out[960+:32] = -45189;
	assign sine_wave_out[928+:32] = -44011;
	assign sine_wave_out[896+:32] = -42806;
	assign sine_wave_out[864+:32] = -41575;
	assign sine_wave_out[832+:32] = -40319;
	assign sine_wave_out[800+:32] = -39039;
	assign sine_wave_out[768+:32] = -37736;
	assign sine_wave_out[736+:32] = -36409;
	assign sine_wave_out[704+:32] = -35061;
	assign sine_wave_out[672+:32] = -33692;
	assign sine_wave_out[640+:32] = -32302;
	assign sine_wave_out[608+:32] = -30893;
	assign sine_wave_out[576+:32] = -29465;
	assign sine_wave_out[544+:32] = -28020;
	assign sine_wave_out[512+:32] = -26557;
	assign sine_wave_out[480+:32] = -25079;
	assign sine_wave_out[448+:32] = -23586;
	assign sine_wave_out[416+:32] = -22078;
	assign sine_wave_out[384+:32] = -20557;
	assign sine_wave_out[352+:32] = -19024;
	assign sine_wave_out[320+:32] = -17479;
	assign sine_wave_out[288+:32] = -15923;
	assign sine_wave_out[256+:32] = -14359;
	assign sine_wave_out[224+:32] = -12785;
	assign sine_wave_out[192+:32] = -11204;
	assign sine_wave_out[160+:32] = -9616;
	assign sine_wave_out[128+:32] = -8022;
	assign sine_wave_out[96+:32] = -6423;
	assign sine_wave_out[64+:32] = -4821;
	assign sine_wave_out[32+:32] = -3215;
	assign sine_wave_out[0+:32] = -1608;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_128VRTL (sine_wave_out);
	output wire [4095:0] sine_wave_out;
	assign sine_wave_out[4064+:32] = 0;
	assign sine_wave_out[4032+:32] = 3215;
	assign sine_wave_out[4000+:32] = 6423;
	assign sine_wave_out[3968+:32] = 9616;
	assign sine_wave_out[3936+:32] = 12785;
	assign sine_wave_out[3904+:32] = 15923;
	assign sine_wave_out[3872+:32] = 19024;
	assign sine_wave_out[3840+:32] = 22078;
	assign sine_wave_out[3808+:32] = 25079;
	assign sine_wave_out[3776+:32] = 28020;
	assign sine_wave_out[3744+:32] = 30893;
	assign sine_wave_out[3712+:32] = 33692;
	assign sine_wave_out[3680+:32] = 36409;
	assign sine_wave_out[3648+:32] = 39039;
	assign sine_wave_out[3616+:32] = 41575;
	assign sine_wave_out[3584+:32] = 44011;
	assign sine_wave_out[3552+:32] = 46340;
	assign sine_wave_out[3520+:32] = 48558;
	assign sine_wave_out[3488+:32] = 50660;
	assign sine_wave_out[3456+:32] = 52639;
	assign sine_wave_out[3424+:32] = 54491;
	assign sine_wave_out[3392+:32] = 56212;
	assign sine_wave_out[3360+:32] = 57797;
	assign sine_wave_out[3328+:32] = 59243;
	assign sine_wave_out[3296+:32] = 60547;
	assign sine_wave_out[3264+:32] = 61705;
	assign sine_wave_out[3232+:32] = 62714;
	assign sine_wave_out[3200+:32] = 63571;
	assign sine_wave_out[3168+:32] = 64276;
	assign sine_wave_out[3136+:32] = 64826;
	assign sine_wave_out[3104+:32] = 65220;
	assign sine_wave_out[3072+:32] = 65457;
	assign sine_wave_out[3040+:32] = 65536;
	assign sine_wave_out[3008+:32] = 65457;
	assign sine_wave_out[2976+:32] = 65220;
	assign sine_wave_out[2944+:32] = 64826;
	assign sine_wave_out[2912+:32] = 64276;
	assign sine_wave_out[2880+:32] = 63571;
	assign sine_wave_out[2848+:32] = 62714;
	assign sine_wave_out[2816+:32] = 61705;
	assign sine_wave_out[2784+:32] = 60547;
	assign sine_wave_out[2752+:32] = 59243;
	assign sine_wave_out[2720+:32] = 57797;
	assign sine_wave_out[2688+:32] = 56212;
	assign sine_wave_out[2656+:32] = 54491;
	assign sine_wave_out[2624+:32] = 52639;
	assign sine_wave_out[2592+:32] = 50660;
	assign sine_wave_out[2560+:32] = 48558;
	assign sine_wave_out[2528+:32] = 46340;
	assign sine_wave_out[2496+:32] = 44011;
	assign sine_wave_out[2464+:32] = 41575;
	assign sine_wave_out[2432+:32] = 39039;
	assign sine_wave_out[2400+:32] = 36409;
	assign sine_wave_out[2368+:32] = 33692;
	assign sine_wave_out[2336+:32] = 30893;
	assign sine_wave_out[2304+:32] = 28020;
	assign sine_wave_out[2272+:32] = 25079;
	assign sine_wave_out[2240+:32] = 22078;
	assign sine_wave_out[2208+:32] = 19024;
	assign sine_wave_out[2176+:32] = 15923;
	assign sine_wave_out[2144+:32] = 12785;
	assign sine_wave_out[2112+:32] = 9616;
	assign sine_wave_out[2080+:32] = 6423;
	assign sine_wave_out[2048+:32] = 3215;
	assign sine_wave_out[2016+:32] = 0;
	assign sine_wave_out[1984+:32] = -3215;
	assign sine_wave_out[1952+:32] = -6423;
	assign sine_wave_out[1920+:32] = -9616;
	assign sine_wave_out[1888+:32] = -12785;
	assign sine_wave_out[1856+:32] = -15923;
	assign sine_wave_out[1824+:32] = -19024;
	assign sine_wave_out[1792+:32] = -22078;
	assign sine_wave_out[1760+:32] = -25079;
	assign sine_wave_out[1728+:32] = -28020;
	assign sine_wave_out[1696+:32] = -30893;
	assign sine_wave_out[1664+:32] = -33692;
	assign sine_wave_out[1632+:32] = -36409;
	assign sine_wave_out[1600+:32] = -39039;
	assign sine_wave_out[1568+:32] = -41575;
	assign sine_wave_out[1536+:32] = -44011;
	assign sine_wave_out[1504+:32] = -46340;
	assign sine_wave_out[1472+:32] = -48558;
	assign sine_wave_out[1440+:32] = -50660;
	assign sine_wave_out[1408+:32] = -52639;
	assign sine_wave_out[1376+:32] = -54491;
	assign sine_wave_out[1344+:32] = -56212;
	assign sine_wave_out[1312+:32] = -57797;
	assign sine_wave_out[1280+:32] = -59243;
	assign sine_wave_out[1248+:32] = -60547;
	assign sine_wave_out[1216+:32] = -61705;
	assign sine_wave_out[1184+:32] = -62714;
	assign sine_wave_out[1152+:32] = -63571;
	assign sine_wave_out[1120+:32] = -64276;
	assign sine_wave_out[1088+:32] = -64826;
	assign sine_wave_out[1056+:32] = -65220;
	assign sine_wave_out[1024+:32] = -65457;
	assign sine_wave_out[992+:32] = -65536;
	assign sine_wave_out[960+:32] = -65457;
	assign sine_wave_out[928+:32] = -65220;
	assign sine_wave_out[896+:32] = -64826;
	assign sine_wave_out[864+:32] = -64276;
	assign sine_wave_out[832+:32] = -63571;
	assign sine_wave_out[800+:32] = -62714;
	assign sine_wave_out[768+:32] = -61705;
	assign sine_wave_out[736+:32] = -60547;
	assign sine_wave_out[704+:32] = -59243;
	assign sine_wave_out[672+:32] = -57797;
	assign sine_wave_out[640+:32] = -56212;
	assign sine_wave_out[608+:32] = -54491;
	assign sine_wave_out[576+:32] = -52639;
	assign sine_wave_out[544+:32] = -50660;
	assign sine_wave_out[512+:32] = -48558;
	assign sine_wave_out[480+:32] = -46340;
	assign sine_wave_out[448+:32] = -44011;
	assign sine_wave_out[416+:32] = -41575;
	assign sine_wave_out[384+:32] = -39039;
	assign sine_wave_out[352+:32] = -36409;
	assign sine_wave_out[320+:32] = -33692;
	assign sine_wave_out[288+:32] = -30893;
	assign sine_wave_out[256+:32] = -28020;
	assign sine_wave_out[224+:32] = -25079;
	assign sine_wave_out[192+:32] = -22078;
	assign sine_wave_out[160+:32] = -19024;
	assign sine_wave_out[128+:32] = -15923;
	assign sine_wave_out[96+:32] = -12785;
	assign sine_wave_out[64+:32] = -9616;
	assign sine_wave_out[32+:32] = -6423;
	assign sine_wave_out[0+:32] = -3215;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_64VRTL (sine_wave_out);
	output wire [2047:0] sine_wave_out;
	assign sine_wave_out[2016+:32] = 0;
	assign sine_wave_out[1984+:32] = 6423;
	assign sine_wave_out[1952+:32] = 12785;
	assign sine_wave_out[1920+:32] = 19024;
	assign sine_wave_out[1888+:32] = 25079;
	assign sine_wave_out[1856+:32] = 30893;
	assign sine_wave_out[1824+:32] = 36409;
	assign sine_wave_out[1792+:32] = 41575;
	assign sine_wave_out[1760+:32] = 46340;
	assign sine_wave_out[1728+:32] = 50660;
	assign sine_wave_out[1696+:32] = 54491;
	assign sine_wave_out[1664+:32] = 57797;
	assign sine_wave_out[1632+:32] = 60547;
	assign sine_wave_out[1600+:32] = 62714;
	assign sine_wave_out[1568+:32] = 64276;
	assign sine_wave_out[1536+:32] = 65220;
	assign sine_wave_out[1504+:32] = 65536;
	assign sine_wave_out[1472+:32] = 65220;
	assign sine_wave_out[1440+:32] = 64276;
	assign sine_wave_out[1408+:32] = 62714;
	assign sine_wave_out[1376+:32] = 60547;
	assign sine_wave_out[1344+:32] = 57797;
	assign sine_wave_out[1312+:32] = 54491;
	assign sine_wave_out[1280+:32] = 50660;
	assign sine_wave_out[1248+:32] = 46340;
	assign sine_wave_out[1216+:32] = 41575;
	assign sine_wave_out[1184+:32] = 36409;
	assign sine_wave_out[1152+:32] = 30893;
	assign sine_wave_out[1120+:32] = 25079;
	assign sine_wave_out[1088+:32] = 19024;
	assign sine_wave_out[1056+:32] = 12785;
	assign sine_wave_out[1024+:32] = 6423;
	assign sine_wave_out[992+:32] = 0;
	assign sine_wave_out[960+:32] = -6423;
	assign sine_wave_out[928+:32] = -12785;
	assign sine_wave_out[896+:32] = -19024;
	assign sine_wave_out[864+:32] = -25079;
	assign sine_wave_out[832+:32] = -30893;
	assign sine_wave_out[800+:32] = -36409;
	assign sine_wave_out[768+:32] = -41575;
	assign sine_wave_out[736+:32] = -46340;
	assign sine_wave_out[704+:32] = -50660;
	assign sine_wave_out[672+:32] = -54491;
	assign sine_wave_out[640+:32] = -57797;
	assign sine_wave_out[608+:32] = -60547;
	assign sine_wave_out[576+:32] = -62714;
	assign sine_wave_out[544+:32] = -64276;
	assign sine_wave_out[512+:32] = -65220;
	assign sine_wave_out[480+:32] = -65536;
	assign sine_wave_out[448+:32] = -65220;
	assign sine_wave_out[416+:32] = -64276;
	assign sine_wave_out[384+:32] = -62714;
	assign sine_wave_out[352+:32] = -60547;
	assign sine_wave_out[320+:32] = -57797;
	assign sine_wave_out[288+:32] = -54491;
	assign sine_wave_out[256+:32] = -50660;
	assign sine_wave_out[224+:32] = -46340;
	assign sine_wave_out[192+:32] = -41575;
	assign sine_wave_out[160+:32] = -36409;
	assign sine_wave_out[128+:32] = -30893;
	assign sine_wave_out[96+:32] = -25079;
	assign sine_wave_out[64+:32] = -19024;
	assign sine_wave_out[32+:32] = -12785;
	assign sine_wave_out[0+:32] = -6423;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_32VRTL (sine_wave_out);
	output wire [1023:0] sine_wave_out;
	assign sine_wave_out[992+:32] = 0;
	assign sine_wave_out[960+:32] = 12785;
	assign sine_wave_out[928+:32] = 25079;
	assign sine_wave_out[896+:32] = 36409;
	assign sine_wave_out[864+:32] = 46340;
	assign sine_wave_out[832+:32] = 54491;
	assign sine_wave_out[800+:32] = 60547;
	assign sine_wave_out[768+:32] = 64276;
	assign sine_wave_out[736+:32] = 65536;
	assign sine_wave_out[704+:32] = 64276;
	assign sine_wave_out[672+:32] = 60547;
	assign sine_wave_out[640+:32] = 54491;
	assign sine_wave_out[608+:32] = 46340;
	assign sine_wave_out[576+:32] = 36409;
	assign sine_wave_out[544+:32] = 25079;
	assign sine_wave_out[512+:32] = 12785;
	assign sine_wave_out[480+:32] = 0;
	assign sine_wave_out[448+:32] = -12785;
	assign sine_wave_out[416+:32] = -25079;
	assign sine_wave_out[384+:32] = -36409;
	assign sine_wave_out[352+:32] = -46340;
	assign sine_wave_out[320+:32] = -54491;
	assign sine_wave_out[288+:32] = -60547;
	assign sine_wave_out[256+:32] = -64276;
	assign sine_wave_out[224+:32] = -65536;
	assign sine_wave_out[192+:32] = -64276;
	assign sine_wave_out[160+:32] = -60547;
	assign sine_wave_out[128+:32] = -54491;
	assign sine_wave_out[96+:32] = -46340;
	assign sine_wave_out[64+:32] = -36409;
	assign sine_wave_out[32+:32] = -25079;
	assign sine_wave_out[0+:32] = -12785;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_16VRTL (sine_wave_out);
	output wire [511:0] sine_wave_out;
	assign sine_wave_out[480+:32] = 0;
	assign sine_wave_out[448+:32] = 25079;
	assign sine_wave_out[416+:32] = 46340;
	assign sine_wave_out[384+:32] = 60547;
	assign sine_wave_out[352+:32] = 65536;
	assign sine_wave_out[320+:32] = 60547;
	assign sine_wave_out[288+:32] = 46340;
	assign sine_wave_out[256+:32] = 25079;
	assign sine_wave_out[224+:32] = 0;
	assign sine_wave_out[192+:32] = -25079;
	assign sine_wave_out[160+:32] = -46340;
	assign sine_wave_out[128+:32] = -60547;
	assign sine_wave_out[96+:32] = -65536;
	assign sine_wave_out[64+:32] = -60547;
	assign sine_wave_out[32+:32] = -46340;
	assign sine_wave_out[0+:32] = -25079;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_8VRTL (sine_wave_out);
	output wire [255:0] sine_wave_out;
	assign sine_wave_out[224+:32] = 0;
	assign sine_wave_out[192+:32] = 46340;
	assign sine_wave_out[160+:32] = 65536;
	assign sine_wave_out[128+:32] = 46340;
	assign sine_wave_out[96+:32] = 0;
	assign sine_wave_out[64+:32] = -46340;
	assign sine_wave_out[32+:32] = -65536;
	assign sine_wave_out[0+:32] = -46340;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_2VRTL (sine_wave_out);
	output wire [63:0] sine_wave_out;
	assign sine_wave_out[32+:32] = 0;
	assign sine_wave_out[0+:32] = 0;
endmodule
module SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_4VRTL (sine_wave_out);
	output wire [127:0] sine_wave_out;
	assign sine_wave_out[96+:32] = 0;
	assign sine_wave_out[64+:32] = 65536;
	assign sine_wave_out[32+:32] = 0;
	assign sine_wave_out[0+:32] = -65536;
endmodule
module CombinationalFFTCrossbarVRTl (
	recv_real,
	recv_imaginary,
	recv_val,
	recv_rdy,
	send_real,
	send_imaginary,
	send_val,
	send_rdy
);
	parameter BIT_WIDTH = 32;
	parameter SIZE_FFT = 8;
	parameter STAGE_FFT = 0;
	parameter FRONT = 1;
	input wire [(SIZE_FFT * BIT_WIDTH) - 1:0] recv_real;
	input wire [(SIZE_FFT * BIT_WIDTH) - 1:0] recv_imaginary;
	input wire [SIZE_FFT - 1:0] recv_val;
	output wire [SIZE_FFT - 1:0] recv_rdy;
	output wire [(SIZE_FFT * BIT_WIDTH) - 1:0] send_real;
	output wire [(SIZE_FFT * BIT_WIDTH) - 1:0] send_imaginary;
	output wire [SIZE_FFT - 1:0] send_val;
	input wire [SIZE_FFT - 1:0] send_rdy;
	genvar m;
	generate
		for (m = 0; m < (2 ** STAGE_FFT); m = m + 1) begin : genblk1
			genvar i;
			for (i = m; i < SIZE_FFT; i = i + (2 ** (STAGE_FFT + 1))) begin : genblk1
				if (FRONT == 1) begin : genblk1
					assign send_real[(i + m) * BIT_WIDTH+:BIT_WIDTH] = recv_real[i * BIT_WIDTH+:BIT_WIDTH];
					assign send_imaginary[(i + m) * BIT_WIDTH+:BIT_WIDTH] = recv_imaginary[i * BIT_WIDTH+:BIT_WIDTH];
					assign send_val[i + m] = recv_val[i];
					assign recv_rdy[i + m] = send_rdy[i];
					assign send_real[((i + m) + 1) * BIT_WIDTH+:BIT_WIDTH] = recv_real[(i + (2 ** STAGE_FFT)) * BIT_WIDTH+:BIT_WIDTH];
					assign send_imaginary[((i + m) + 1) * BIT_WIDTH+:BIT_WIDTH] = recv_imaginary[(i + (2 ** STAGE_FFT)) * BIT_WIDTH+:BIT_WIDTH];
					assign send_val[(i + m) + 1] = recv_val[i + (2 ** STAGE_FFT)];
					assign recv_rdy[(i + m) + 1] = send_rdy[i + (2 ** STAGE_FFT)];
				end
				else begin : genblk1
					assign send_real[i * BIT_WIDTH+:BIT_WIDTH] = recv_real[(i + m) * BIT_WIDTH+:BIT_WIDTH];
					assign send_imaginary[i * BIT_WIDTH+:BIT_WIDTH] = recv_imaginary[(i + m) * BIT_WIDTH+:BIT_WIDTH];
					assign send_val[i] = recv_val[i + m];
					assign recv_rdy[i] = send_rdy[i + m];
					assign send_real[(i + (2 ** STAGE_FFT)) * BIT_WIDTH+:BIT_WIDTH] = recv_real[((i + m) + 1) * BIT_WIDTH+:BIT_WIDTH];
					assign send_imaginary[(i + (2 ** STAGE_FFT)) * BIT_WIDTH+:BIT_WIDTH] = recv_imaginary[((i + m) + 1) * BIT_WIDTH+:BIT_WIDTH];
					assign send_val[i + (2 ** STAGE_FFT)] = recv_val[(i + m) + 1];
					assign recv_rdy[i + (2 ** STAGE_FFT)] = send_rdy[(i + m) + 1];
				end
			end
		end
	endgenerate
endmodule
module RegisterV_Reset (
	clk,
	reset,
	w,
	d,
	q
);
	parameter N = 8;
	input wire clk;
	input wire reset;
	input wire w;
	input wire [N - 1:0] d;
	output wire [N - 1:0] q;
	reg [N - 1:0] regout;
	assign q = regout;
	always @(posedge clk)
		if (reset)
			regout <= 0;
		else if (w)
			regout <= d;
endmodule
module FpmultVRTL (
	clk,
	reset,
	recv_val,
	recv_rdy,
	send_val,
	send_rdy,
	a,
	b,
	c
);
	parameter n = 32;
	parameter d = 16;
	parameter sign = 1;
	input wire clk;
	input wire reset;
	input wire recv_val;
	input wire send_rdy;
	input wire [n - 1:0] a;
	input wire [n - 1:0] b;
	output wire [n - 1:0] c;
	output wire send_val;
	output wire recv_rdy;
	wire do_carry;
	wire do_add;
	wire in_wait;
	fpmult_control #(
		.n(n),
		.d(d)
	) control(
		.clk(clk),
		.reset(reset),
		.recv_val(recv_val),
		.recv_rdy(recv_rdy),
		.send_val(send_val),
		.send_rdy(send_rdy),
		.in_wait(in_wait),
		.do_add(do_add),
		.do_carry(do_carry)
	);
	fpmult_datapath #(
		.n(n),
		.d(d)
	) datapath(
		.clk(clk),
		.reset(reset),
		.in_wait(in_wait),
		.do_add(do_add),
		.do_carry((sign != 0) & do_carry),
		.a({{d {(sign != 0) & a[n - 1]}}, a}),
		.b(b),
		.c(c)
	);
endmodule
module fpmult_control (
	clk,
	reset,
	recv_val,
	recv_rdy,
	send_val,
	send_rdy,
	in_wait,
	do_add,
	do_carry
);
	parameter n = 0;
	parameter d = 0;
	input wire clk;
	input wire reset;
	input wire recv_val;
	output reg recv_rdy;
	output reg send_val;
	input wire send_rdy;
	output reg in_wait;
	output reg do_add;
	output reg do_carry;
	localparam [1:0] IDLE = 2'd0;
	localparam [1:0] CALC = 2'd1;
	localparam [1:0] DONE = 2'd2;
	reg [1:0] state;
	reg [1:0] next_state;
	reg [$clog2(n) - 1:0] counter;
	reg counter_reset;
	function automatic signed [$clog2(n) - 1:0] sv2v_cast_2A747_signed;
		input reg signed [$clog2(n) - 1:0] inp;
		sv2v_cast_2A747_signed = inp;
	endfunction
	always @(*)
		case (state)
			IDLE:
				if (recv_val)
					next_state = CALC;
				else
					next_state = IDLE;
			CALC:
				if (counter == sv2v_cast_2A747_signed(n - 1))
					next_state = DONE;
				else
					next_state = CALC;
			DONE:
				if (send_rdy)
					next_state = IDLE;
				else
					next_state = DONE;
			default: next_state = IDLE;
		endcase
	always @(*)
		case (state)
			IDLE: begin
				in_wait = 1;
				do_add = 0;
				do_carry = 0;
				counter_reset = 0;
				recv_rdy = 1;
				send_val = 0;
			end
			CALC: begin
				in_wait = 0;
				do_add = 1;
				do_carry = counter == sv2v_cast_2A747_signed(n - 1);
				counter_reset = 0;
				recv_rdy = 0;
				send_val = 0;
			end
			DONE: begin
				in_wait = 0;
				do_add = 0;
				do_carry = 0;
				counter_reset = 1;
				recv_rdy = 0;
				send_val = 1;
			end
			default:
				;
		endcase
	always @(posedge clk)
		if (reset)
			state <= IDLE;
		else
			state <= next_state;
	always @(posedge clk)
		if (reset || counter_reset)
			counter <= 0;
		else if (state == CALC)
			counter <= counter + 1;
		else
			counter <= counter;
endmodule
module fpmult_datapath (
	clk,
	reset,
	in_wait,
	do_add,
	do_carry,
	a,
	b,
	c
);
	parameter n = 0;
	parameter d = 0;
	input wire clk;
	input wire reset;
	input wire in_wait;
	input wire do_add;
	input wire do_carry;
	input wire [(n + d) - 1:0] a;
	input wire [n - 1:0] b;
	output wire [n - 1:0] c;
	wire [(n + d) - 1:0] acc_in;
	wire [(n + d) - 1:0] acc_out;
	RegisterV_Reset #(.N(n + d)) acc_reg(
		.clk(clk),
		.reset(in_wait | reset),
		.w(1),
		.d(acc_in),
		.q(acc_out)
	);
	wire [(n + d) - 1:0] a_const_out;
	RegisterV_Reset #(.N(n + d)) a_const_reg(
		.clk(clk),
		.reset(reset),
		.w(in_wait),
		.d(a),
		.q(a_const_out)
	);
	wire [(n + d) - 1:0] a_in;
	wire [(n + d) - 1:0] a_out;
	RegisterV_Reset #(.N(n + d)) a_reg(
		.clk(clk),
		.reset(reset),
		.w(1),
		.d(a_in),
		.q(a_out)
	);
	wire [(n + d) - 1:0] b_in;
	wire [(n + d) - 1:0] b_out;
	RegisterV_Reset #(.N(n)) b_reg(
		.clk(clk),
		.reset(reset),
		.w(1),
		.d(b_in),
		.q(b_out)
	);
	vc_Mux2 #(.p_nbits(n + d)) a_sel(
		.in0(a_out << 1),
		.in1(a),
		.sel(in_wait),
		.out(a_in)
	);
	vc_Mux2 #(.p_nbits(n)) b_sel(
		.in0(b_out >> 1),
		.in1(b),
		.sel(in_wait),
		.out(b_in)
	);
	wire [(n + d) - 1:0] add_tmp;
	wire [(n + d) - 1:0] carry;
	wire [(2 * n) - 1:0] carry_tmp;
	wire [(2 * n) - 1:0] carry_tmp2;
	assign carry_tmp = {{n - d {a_const_out[(n + d) - 1]}}, a_const_out};
	assign carry_tmp2 = ((carry_tmp << n) - carry_tmp) << (n - 1);
	vc_Mux2 #(.p_nbits(n + d)) carry_sel(
		.in0(a_out),
		.in1(carry_tmp2[(n + d) - 1:0]),
		.sel(do_carry),
		.out(add_tmp)
	);
	vc_Mux2 #(.p_nbits(n + d)) add_sel(
		.in0(acc_out),
		.in1(acc_out + add_tmp),
		.sel(do_add & b_out[0]),
		.out(acc_in)
	);
	assign c = acc_out[(n + d) - 1:d];
endmodule
module FpcmultVRTL (
	clk,
	reset,
	recv_val,
	recv_rdy,
	send_val,
	send_rdy,
	ar,
	ac,
	br,
	bc,
	cr,
	cc
);
	parameter n = 32;
	parameter d = 16;
	input wire clk;
	input wire reset;
	input wire recv_val;
	output wire recv_rdy;
	output wire send_val;
	input wire send_rdy;
	input wire [n - 1:0] ar;
	input wire [n - 1:0] ac;
	input wire [n - 1:0] br;
	input wire [n - 1:0] bc;
	output wire [n - 1:0] cr;
	output wire [n - 1:0] cc;
	wire [n - 1:0] arbr;
	wire [n - 1:0] acbc;
	wire [n - 1:0] ar_plus_ac;
	wire [n - 1:0] br_plus_bc;
	wire [n - 1:0] ab;
	assign ar_plus_ac = ar + ac;
	assign br_plus_bc = br + bc;
	wire recv_rdy_imm [2:0];
	assign recv_rdy = (recv_rdy_imm[0] & recv_rdy_imm[1]) & recv_rdy_imm[2];
	wire send_val_imm [2:0];
	assign send_val = (send_val_imm[0] & send_val_imm[1]) & send_val_imm[2];
	FpmultVRTL #(
		.n(n),
		.d(d),
		.sign(1)
	) m1(
		.clk(clk),
		.reset(reset),
		.a(ar),
		.b(br),
		.c(arbr),
		.recv_val(recv_val),
		.recv_rdy(recv_rdy_imm[0]),
		.send_val(send_val_imm[0]),
		.send_rdy(send_rdy)
	);
	FpmultVRTL #(
		.n(n),
		.d(d),
		.sign(1)
	) m2(
		.clk(clk),
		.reset(reset),
		.a(ac),
		.b(bc),
		.c(acbc),
		.recv_val(recv_val),
		.recv_rdy(recv_rdy_imm[1]),
		.send_val(send_val_imm[1]),
		.send_rdy(send_rdy)
	);
	FpmultVRTL #(
		.n(n),
		.d(d),
		.sign(1)
	) m3(
		.clk(clk),
		.reset(reset),
		.a(ar_plus_ac),
		.b(br_plus_bc),
		.c(ab),
		.recv_val(recv_val),
		.recv_rdy(recv_rdy_imm[2]),
		.send_val(send_val_imm[2]),
		.send_rdy(send_rdy)
	);
	assign cr = arbr - acbc;
	assign cc = (ab - arbr) - acbc;
endmodule
module ButterflyVRTL (
	clk,
	reset,
	recv_val,
	recv_rdy,
	send_val,
	send_rdy,
	ar,
	ac,
	br,
	bc,
	wr,
	wc,
	cr,
	cc,
	dr,
	dc
);
	parameter n = 32;
	parameter d = 16;
	parameter mult = 0;
	input wire clk;
	input wire reset;
	input wire recv_val;
	input wire send_rdy;
	input wire [n - 1:0] ar;
	input wire [n - 1:0] ac;
	input wire [n - 1:0] br;
	input wire [n - 1:0] bc;
	input wire [n - 1:0] wr;
	input wire [n - 1:0] wc;
	output reg send_val;
	output wire recv_rdy;
	output reg [n - 1:0] cr;
	output reg [n - 1:0] cc;
	output reg [n - 1:0] dr;
	output reg [n - 1:0] dc;
	wire [n - 1:0] ar_imm;
	wire [n - 1:0] ac_imm;
	wire [n - 1:0] tr;
	wire [n - 1:0] tc;
	generate
		case (mult)
			1: begin : genblk1
				always @(posedge clk)
					if (reset) begin
						cr <= 0;
						cc <= 0;
						dr <= 0;
						dc <= 0;
						send_val <= 0;
					end
					else if (recv_val & recv_rdy) begin
						cr <= ar + br;
						cc <= ac + bc;
						dr <= ar - br;
						dc <= ac - bc;
						send_val <= 1;
					end
					else if (send_val & send_rdy)
						send_val <= 0;
					else begin
						cr <= cr;
						cc <= cc;
						dr <= dr;
						dc <= dc;
						send_val <= send_val;
					end
				assign recv_rdy = ~send_val;
			end
			2: begin : genblk1
				always @(posedge clk)
					if (reset) begin
						cr <= 0;
						cc <= 0;
						dr <= 0;
						dc <= 0;
						send_val <= 0;
					end
					else if (recv_val & recv_rdy) begin
						cr <= ar - br;
						cc <= ac - bc;
						dr <= ar + br;
						dc <= ac + bc;
						send_val <= 1;
					end
					else if (send_val & send_rdy)
						send_val <= 0;
					else begin
						cr <= cr;
						cc <= cc;
						dr <= dr;
						dc <= dc;
						send_val <= send_val;
					end
				assign recv_rdy = ~send_val;
			end
			3: begin : genblk1
				always @(posedge clk)
					if (reset) begin
						cr <= 0;
						cc <= 0;
						dr <= 0;
						dc <= 0;
						send_val <= 0;
					end
					else if (recv_val & recv_rdy) begin
						cr <= ar - bc;
						cc <= ac + br;
						dr <= ar + bc;
						dc <= ac - br;
						send_val <= 1;
					end
					else if (send_val & send_rdy)
						send_val <= 0;
					else begin
						cr <= cr;
						cc <= cc;
						dr <= dr;
						dc <= dc;
						send_val <= send_val;
					end
				assign recv_rdy = ~send_val;
			end
			4: begin : genblk1
				always @(posedge clk)
					if (reset) begin
						cr <= 0;
						cc <= 0;
						dr <= 0;
						dc <= 0;
						send_val <= 0;
					end
					else if (recv_val & recv_rdy) begin
						cr <= ar + bc;
						cc <= ac - br;
						dr <= ar - bc;
						dc <= ac + br;
						send_val <= 1;
					end
					else if (send_val & send_rdy)
						send_val <= 0;
					else begin
						cr <= cr;
						cc <= cc;
						dr <= dr;
						dc <= dc;
						send_val <= send_val;
					end
				assign recv_rdy = ~send_val;
			end
			default: begin : genblk1
				wire [1:1] sv2v_tmp_mul_send_val;
				always @(*) send_val = sv2v_tmp_mul_send_val;
				FpcmultVRTL #(
					.n(n),
					.d(d)
				) mul(
					.clk(clk),
					.reset(reset),
					.ar(br),
					.ac(bc),
					.br(wr),
					.bc(wc),
					.cr(tr),
					.cc(tc),
					.recv_val(recv_val),
					.recv_rdy(recv_rdy),
					.send_val(sv2v_tmp_mul_send_val),
					.send_rdy(send_rdy)
				);
				vc_EnResetReg #(.p_nbits(n)) ac_reg(
					.clk(clk),
					.en(recv_rdy),
					.d(ac),
					.q(ac_imm),
					.reset(reset)
				);
				vc_EnResetReg #(.p_nbits(n)) ar_reg(
					.clk(clk),
					.en(recv_rdy),
					.d(ar),
					.q(ar_imm),
					.reset(reset)
				);
				wire [n:1] sv2v_tmp_290D2;
				assign sv2v_tmp_290D2 = ar_imm + tr;
				always @(*) cr = sv2v_tmp_290D2;
				wire [n:1] sv2v_tmp_BBFA5;
				assign sv2v_tmp_BBFA5 = ac_imm + tc;
				always @(*) cc = sv2v_tmp_BBFA5;
				wire [n:1] sv2v_tmp_EF039;
				assign sv2v_tmp_EF039 = ar_imm - tr;
				always @(*) dr = sv2v_tmp_EF039;
				wire [n:1] sv2v_tmp_CFFA2;
				assign sv2v_tmp_CFFA2 = ac_imm - tc;
				always @(*) dc = sv2v_tmp_CFFA2;
			end
		endcase
	endgenerate
endmodule
module FFT_StageVRTL (
	recv_msg_real,
	recv_msg_imag,
	recv_val,
	recv_rdy,
	send_msg_real,
	send_msg_imag,
	send_val,
	send_rdy,
	sine_wave_out,
	reset,
	clk
);
	parameter BIT_WIDTH = 32;
	parameter DECIMAL_PT = 16;
	parameter N_SAMPLES = 8;
	parameter STAGE_FFT = 0;
	input wire [(N_SAMPLES * BIT_WIDTH) - 1:0] recv_msg_real;
	input wire [(N_SAMPLES * BIT_WIDTH) - 1:0] recv_msg_imag;
	input wire recv_val;
	output wire recv_rdy;
	output wire [(N_SAMPLES * BIT_WIDTH) - 1:0] send_msg_real;
	output wire [(N_SAMPLES * BIT_WIDTH) - 1:0] send_msg_imag;
	output wire send_val;
	input wire send_rdy;
	input wire [(N_SAMPLES * BIT_WIDTH) - 1:0] sine_wave_out;
	input wire reset;
	input wire clk;
	wire [N_SAMPLES - 1:0] val_in;
	wire [N_SAMPLES - 1:0] rdy_in;
	wire [N_SAMPLES - 1:0] val_out;
	wire [N_SAMPLES - 1:0] rdy_out;
	wire [N_SAMPLES - 1:0] imm;
	genvar i;
	generate
		for (i = 0; i < N_SAMPLES; i = i + 1) begin : genblk1
			assign val_in[i] = recv_val;
			assign imm[i] = rdy_in[i];
		end
	endgenerate
	assign recv_rdy = imm == {N_SAMPLES {1'b1}};
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] butterfly_in_real;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] butterfly_out_real;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] butterfly_in_imaginary;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] butterfly_out_imaginary;
	wire [N_SAMPLES - 1:0] val_interior_in;
	wire [N_SAMPLES - 1:0] rdy_interior_in;
	wire [N_SAMPLES - 1:0] val_interior_out;
	wire [N_SAMPLES - 1:0] rdy_interior_out;
	wire [((N_SAMPLES / 2) * BIT_WIDTH) - 1:0] twiddle_real;
	wire [((N_SAMPLES / 2) * BIT_WIDTH) - 1:0] twiddle_imaginary;
	wire val_interior_mini [(N_SAMPLES / 2) - 1:0];
	wire rdy_interior_mini [(N_SAMPLES / 2) - 1:0];
	CombinationalFFTCrossbarVRTl #(
		.BIT_WIDTH(BIT_WIDTH),
		.SIZE_FFT(N_SAMPLES),
		.STAGE_FFT(STAGE_FFT),
		.FRONT(1)
	) xbar_in_1(
		.recv_real(recv_msg_real),
		.recv_imaginary(recv_msg_imag),
		.recv_val(val_in),
		.recv_rdy(rdy_in),
		.send_real(butterfly_in_real[BIT_WIDTH * ((N_SAMPLES - 1) - (N_SAMPLES - 1))+:BIT_WIDTH * N_SAMPLES]),
		.send_imaginary(butterfly_in_imaginary[BIT_WIDTH * ((N_SAMPLES - 1) - (N_SAMPLES - 1))+:BIT_WIDTH * N_SAMPLES]),
		.send_val(val_interior_in),
		.send_rdy(rdy_interior_in)
	);
	genvar b;
	generate
		for (b = 0; b < (N_SAMPLES / 2); b = b + 1) begin : genblk2
			localparam IX = (b % (1 << STAGE_FFT)) * (N_SAMPLES / (2 * (1 << STAGE_FFT)));
			localparam MMC = (IX == 0 ? 1 : (IX == (N_SAMPLES >> 1) ? 2 : (IX == (N_SAMPLES >> 2) ? 4 : (IX == (3 * (N_SAMPLES >> 2)) ? 3 : 0))));
			ButterflyVRTL #(
				.n(BIT_WIDTH),
				.d(DECIMAL_PT),
				.mult(MMC)
			) bfu_in(
				.ar(butterfly_in_real[(b * 2) * BIT_WIDTH+:BIT_WIDTH]),
				.ac(butterfly_in_imaginary[(b * 2) * BIT_WIDTH+:BIT_WIDTH]),
				.br(butterfly_in_real[((b * 2) + 1) * BIT_WIDTH+:BIT_WIDTH]),
				.bc(butterfly_in_imaginary[((b * 2) + 1) * BIT_WIDTH+:BIT_WIDTH]),
				.wr(twiddle_real[b * BIT_WIDTH+:BIT_WIDTH]),
				.wc(twiddle_imaginary[b * BIT_WIDTH+:BIT_WIDTH]),
				.recv_val(val_interior_in[b * 2] && val_interior_in[(b * 2) + 1]),
				.recv_rdy(rdy_interior_mini[b]),
				.cr(butterfly_out_real[(b * 2) * BIT_WIDTH+:BIT_WIDTH]),
				.cc(butterfly_out_imaginary[(b * 2) * BIT_WIDTH+:BIT_WIDTH]),
				.dr(butterfly_out_real[((b * 2) + 1) * BIT_WIDTH+:BIT_WIDTH]),
				.dc(butterfly_out_imaginary[((b * 2) + 1) * BIT_WIDTH+:BIT_WIDTH]),
				.send_rdy((send_val && rdy_interior_out[b * 2]) && rdy_interior_out[(b * 2) + 1]),
				.send_val(val_interior_mini[b]),
				.reset(reset),
				.clk(clk)
			);
			assign val_interior_out[(b * 2) + 1] = val_interior_mini[b];
			assign val_interior_out[b * 2] = val_interior_mini[b];
			assign rdy_interior_in[(b * 2) + 1] = rdy_interior_mini[b];
			assign rdy_interior_in[b * 2] = rdy_interior_mini[b];
		end
	endgenerate
	CombinationalFFTCrossbarVRTl #(
		.BIT_WIDTH(BIT_WIDTH),
		.SIZE_FFT(N_SAMPLES),
		.STAGE_FFT(STAGE_FFT),
		.FRONT(0)
	) xbar_out_1(
		.recv_real(butterfly_out_real),
		.recv_imaginary(butterfly_out_imaginary),
		.recv_val(val_interior_out),
		.recv_rdy(rdy_interior_out),
		.send_real(send_msg_real),
		.send_imaginary(send_msg_imag),
		.send_val(val_out),
		.send_rdy(rdy_out)
	);
	TwiddleGeneratorVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.DECIMAL_PT(DECIMAL_PT),
		.SIZE_FFT(N_SAMPLES),
		.STAGE_FFT(STAGE_FFT)
	) twiddle_generator(
		.sine_wave_in(sine_wave_out),
		.twiddle_real(twiddle_real),
		.twiddle_imaginary(twiddle_imaginary)
	);
	wire [N_SAMPLES - 1:0] imm2;
	generate
		for (i = 0; i < N_SAMPLES; i = i + 1) begin : genblk3
			assign imm2[i] = val_out[i];
			assign rdy_out[i] = send_rdy & send_val;
		end
	endgenerate
	assign send_val = imm2 == {N_SAMPLES {1'b1}};
endmodule
module FFTVRTL (
	recv_msg,
	recv_val,
	recv_rdy,
	send_msg,
	send_val,
	send_rdy,
	reset,
	clk
);
	parameter BIT_WIDTH = 32;
	parameter DECIMAL_PT = 16;
	parameter N_SAMPLES = 8;
	input wire [(N_SAMPLES * BIT_WIDTH) - 1:0] recv_msg;
	input wire recv_val;
	output wire recv_rdy;
	output reg [(N_SAMPLES * BIT_WIDTH) - 1:0] send_msg;
	output wire send_val;
	input wire send_rdy;
	input wire reset;
	input wire clk;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] real_msg [$clog2(N_SAMPLES):0];
	reg [(N_SAMPLES * BIT_WIDTH) - 1:0] complex_msg [$clog2(N_SAMPLES):0];
	wire val_in [$clog2(N_SAMPLES):0];
	wire rdy_in [$clog2(N_SAMPLES):0];
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] sine_wave_out;
	assign val_in[0] = recv_val;
	assign recv_rdy = rdy_in[0];
	assign send_val = val_in[$clog2(N_SAMPLES)];
	assign rdy_in[$clog2(N_SAMPLES)] = send_rdy;
	always @(*) begin : sv2v_autoblock_1
		reg signed [31:0] i;
		for (i = 0; i < N_SAMPLES; i = i + 1)
			complex_msg[0][i * BIT_WIDTH+:BIT_WIDTH] = 0;
	end
	generate
		if (N_SAMPLES == 512) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][256 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][128 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][384 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][64 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[4 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][320 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[5 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][192 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[6 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][448 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[7 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][32 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[8 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][288 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[9 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][160 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[10 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][416 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[11 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][96 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[12 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][352 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[13 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][224 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[14 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][480 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[15 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][16 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[16 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][272 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[17 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][144 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[18 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][400 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[19 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][80 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[20 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][336 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[21 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][208 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[22 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][464 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[23 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][48 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[24 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][304 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[25 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][176 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[26 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][432 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[27 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][112 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[28 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][368 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[29 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][240 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[30 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][496 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[31 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][8 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[32 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][264 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[33 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][136 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[34 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][392 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[35 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][72 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[36 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][328 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[37 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][200 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[38 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][456 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[39 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][40 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[40 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][296 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[41 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][168 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[42 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][424 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[43 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][104 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[44 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][360 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[45 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][232 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[46 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][488 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[47 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][24 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[48 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][280 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[49 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][152 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[50 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][408 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[51 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][88 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[52 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][344 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[53 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][216 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[54 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][472 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[55 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][56 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[56 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][312 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[57 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][184 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[58 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][440 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[59 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][120 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[60 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][376 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[61 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][248 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[62 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][504 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[63 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][4 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[64 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][260 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[65 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][132 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[66 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][388 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[67 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][68 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[68 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][324 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[69 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][196 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[70 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][452 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[71 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][36 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[72 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][292 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[73 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][164 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[74 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][420 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[75 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][100 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[76 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][356 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[77 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][228 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[78 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][484 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[79 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][20 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[80 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][276 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[81 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][148 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[82 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][404 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[83 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][84 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[84 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][340 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[85 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][212 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[86 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][468 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[87 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][52 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[88 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][308 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[89 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][180 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[90 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][436 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[91 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][116 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[92 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][372 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[93 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][244 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[94 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][500 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[95 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][12 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[96 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][268 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[97 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][140 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[98 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][396 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[99 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][76 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[100 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][332 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[101 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][204 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[102 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][460 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[103 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][44 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[104 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][300 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[105 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][172 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[106 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][428 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[107 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][108 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[108 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][364 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[109 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][236 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[110 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][492 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[111 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][28 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[112 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][284 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[113 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][156 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[114 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][412 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[115 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][92 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[116 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][348 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[117 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][220 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[118 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][476 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[119 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][60 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[120 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][316 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[121 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][188 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[122 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][444 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[123 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][124 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[124 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][380 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[125 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][252 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[126 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][508 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[127 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[128 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][258 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[129 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][130 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[130 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][386 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[131 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][66 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[132 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][322 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[133 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][194 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[134 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][450 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[135 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][34 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[136 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][290 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[137 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][162 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[138 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][418 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[139 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][98 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[140 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][354 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[141 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][226 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[142 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][482 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[143 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][18 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[144 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][274 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[145 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][146 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[146 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][402 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[147 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][82 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[148 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][338 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[149 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][210 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[150 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][466 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[151 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][50 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[152 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][306 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[153 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][178 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[154 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][434 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[155 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][114 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[156 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][370 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[157 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][242 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[158 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][498 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[159 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][10 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[160 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][266 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[161 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][138 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[162 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][394 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[163 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][74 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[164 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][330 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[165 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][202 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[166 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][458 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[167 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][42 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[168 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][298 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[169 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][170 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[170 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][426 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[171 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][106 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[172 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][362 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[173 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][234 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[174 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][490 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[175 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][26 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[176 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][282 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[177 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][154 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[178 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][410 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[179 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][90 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[180 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][346 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[181 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][218 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[182 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][474 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[183 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][58 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[184 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][314 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[185 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][186 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[186 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][442 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[187 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][122 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[188 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][378 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[189 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][250 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[190 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][506 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[191 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][6 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[192 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][262 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[193 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][134 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[194 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][390 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[195 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][70 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[196 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][326 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[197 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][198 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[198 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][454 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[199 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][38 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[200 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][294 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[201 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][166 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[202 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][422 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[203 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][102 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[204 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][358 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[205 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][230 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[206 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][486 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[207 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][22 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[208 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][278 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[209 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][150 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[210 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][406 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[211 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][86 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[212 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][342 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[213 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][214 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[214 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][470 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[215 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][54 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[216 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][310 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[217 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][182 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[218 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][438 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[219 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][118 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[220 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][374 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[221 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][246 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[222 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][502 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[223 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][14 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[224 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][270 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[225 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][142 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[226 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][398 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[227 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][78 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[228 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][334 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[229 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][206 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[230 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][462 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[231 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][46 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[232 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][302 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[233 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][174 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[234 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][430 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[235 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][110 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[236 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][366 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[237 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][238 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[238 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][494 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[239 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][30 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[240 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][286 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[241 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][158 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[242 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][414 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[243 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][94 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[244 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][350 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[245 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][222 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[246 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][478 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[247 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][62 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[248 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][318 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[249 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][190 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[250 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][446 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[251 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][126 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[252 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][382 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[253 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][254 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[254 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][510 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[255 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[256 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][257 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[257 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][129 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[258 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][385 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[259 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][65 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[260 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][321 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[261 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][193 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[262 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][449 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[263 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][33 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[264 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][289 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[265 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][161 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[266 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][417 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[267 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][97 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[268 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][353 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[269 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][225 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[270 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][481 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[271 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][17 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[272 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][273 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[273 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][145 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[274 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][401 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[275 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][81 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[276 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][337 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[277 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][209 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[278 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][465 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[279 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][49 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[280 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][305 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[281 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][177 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[282 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][433 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[283 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][113 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[284 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][369 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[285 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][241 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[286 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][497 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[287 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][9 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[288 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][265 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[289 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][137 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[290 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][393 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[291 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][73 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[292 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][329 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[293 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][201 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[294 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][457 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[295 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][41 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[296 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][297 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[297 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][169 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[298 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][425 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[299 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][105 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[300 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][361 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[301 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][233 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[302 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][489 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[303 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][25 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[304 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][281 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[305 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][153 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[306 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][409 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[307 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][89 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[308 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][345 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[309 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][217 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[310 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][473 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[311 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][57 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[312 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][313 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[313 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][185 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[314 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][441 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[315 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][121 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[316 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][377 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[317 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][249 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[318 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][505 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[319 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][5 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[320 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][261 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[321 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][133 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[322 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][389 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[323 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][69 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[324 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][325 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[325 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][197 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[326 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][453 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[327 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][37 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[328 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][293 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[329 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][165 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[330 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][421 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[331 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][101 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[332 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][357 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[333 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][229 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[334 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][485 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[335 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][21 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[336 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][277 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[337 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][149 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[338 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][405 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[339 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][85 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[340 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][341 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[341 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][213 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[342 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][469 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[343 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][53 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[344 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][309 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[345 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][181 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[346 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][437 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[347 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][117 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[348 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][373 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[349 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][245 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[350 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][501 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[351 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][13 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[352 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][269 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[353 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][141 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[354 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][397 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[355 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][77 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[356 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][333 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[357 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][205 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[358 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][461 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[359 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][45 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[360 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][301 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[361 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][173 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[362 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][429 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[363 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][109 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[364 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][365 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[365 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][237 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[366 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][493 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[367 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][29 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[368 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][285 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[369 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][157 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[370 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][413 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[371 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][93 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[372 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][349 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[373 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][221 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[374 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][477 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[375 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][61 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[376 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][317 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[377 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][189 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[378 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][445 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[379 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][125 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[380 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][381 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[381 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][253 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[382 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][509 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[383 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[384 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][259 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[385 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][131 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[386 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][387 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[387 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][67 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[388 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][323 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[389 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][195 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[390 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][451 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[391 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][35 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[392 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][291 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[393 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][163 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[394 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][419 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[395 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][99 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[396 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][355 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[397 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][227 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[398 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][483 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[399 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][19 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[400 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][275 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[401 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][147 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[402 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][403 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[403 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][83 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[404 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][339 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[405 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][211 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[406 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][467 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[407 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][51 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[408 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][307 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[409 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][179 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[410 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][435 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[411 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][115 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[412 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][371 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[413 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][243 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[414 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][499 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[415 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][11 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[416 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][267 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[417 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][139 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[418 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][395 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[419 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][75 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[420 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][331 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[421 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][203 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[422 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][459 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[423 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][43 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[424 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][299 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[425 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][171 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[426 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][427 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[427 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][107 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[428 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][363 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[429 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][235 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[430 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][491 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[431 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][27 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[432 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][283 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[433 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][155 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[434 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][411 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[435 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][91 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[436 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][347 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[437 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][219 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[438 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][475 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[439 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][59 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[440 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][315 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[441 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][187 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[442 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][443 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[443 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][123 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[444 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][379 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[445 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][251 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[446 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][507 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[447 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][7 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[448 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][263 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[449 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][135 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[450 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][391 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[451 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][71 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[452 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][327 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[453 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][199 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[454 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][455 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[455 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][39 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[456 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][295 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[457 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][167 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[458 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][423 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[459 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][103 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[460 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][359 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[461 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][231 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[462 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][487 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[463 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][23 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[464 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][279 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[465 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][151 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[466 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][407 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[467 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][87 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[468 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][343 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[469 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][215 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[470 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][471 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[471 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][55 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[472 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][311 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[473 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][183 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[474 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][439 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[475 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][119 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[476 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][375 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[477 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][247 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[478 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][503 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[479 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][15 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[480 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][271 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[481 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][143 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[482 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][399 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[483 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][79 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[484 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][335 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[485 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][207 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[486 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][463 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[487 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][47 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[488 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][303 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[489 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][175 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[490 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][431 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[491 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][111 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[492 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][367 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[493 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][239 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[494 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][495 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[495 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][31 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[496 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][287 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[497 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][159 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[498 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][415 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[499 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][95 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[500 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][351 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[501 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][223 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[502 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][479 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[503 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][63 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[504 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][319 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[505 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][191 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[506 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][447 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[507 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][127 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[508 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][383 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[509 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][255 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[510 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][511 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[511 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_512VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 256) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][128 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][64 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][192 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][32 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[4 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][160 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[5 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][96 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[6 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][224 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[7 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][16 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[8 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][144 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[9 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][80 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[10 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][208 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[11 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][48 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[12 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][176 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[13 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][112 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[14 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][240 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[15 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][8 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[16 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][136 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[17 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][72 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[18 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][200 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[19 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][40 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[20 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][168 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[21 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][104 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[22 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][232 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[23 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][24 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[24 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][152 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[25 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][88 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[26 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][216 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[27 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][56 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[28 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][184 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[29 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][120 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[30 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][248 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[31 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][4 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[32 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][132 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[33 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][68 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[34 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][196 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[35 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][36 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[36 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][164 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[37 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][100 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[38 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][228 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[39 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][20 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[40 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][148 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[41 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][84 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[42 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][212 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[43 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][52 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[44 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][180 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[45 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][116 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[46 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][244 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[47 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][12 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[48 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][140 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[49 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][76 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[50 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][204 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[51 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][44 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[52 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][172 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[53 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][108 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[54 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][236 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[55 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][28 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[56 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][156 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[57 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][92 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[58 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][220 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[59 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][60 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[60 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][188 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[61 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][124 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[62 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][252 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[63 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[64 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][130 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[65 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][66 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[66 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][194 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[67 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][34 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[68 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][162 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[69 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][98 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[70 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][226 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[71 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][18 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[72 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][146 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[73 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][82 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[74 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][210 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[75 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][50 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[76 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][178 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[77 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][114 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[78 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][242 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[79 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][10 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[80 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][138 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[81 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][74 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[82 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][202 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[83 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][42 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[84 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][170 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[85 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][106 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[86 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][234 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[87 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][26 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[88 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][154 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[89 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][90 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[90 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][218 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[91 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][58 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[92 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][186 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[93 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][122 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[94 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][250 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[95 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][6 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[96 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][134 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[97 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][70 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[98 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][198 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[99 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][38 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[100 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][166 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[101 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][102 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[102 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][230 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[103 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][22 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[104 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][150 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[105 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][86 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[106 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][214 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[107 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][54 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[108 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][182 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[109 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][118 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[110 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][246 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[111 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][14 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[112 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][142 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[113 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][78 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[114 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][206 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[115 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][46 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[116 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][174 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[117 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][110 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[118 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][238 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[119 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][30 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[120 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][158 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[121 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][94 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[122 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][222 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[123 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][62 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[124 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][190 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[125 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][126 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[126 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][254 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[127 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[128 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][129 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[129 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][65 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[130 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][193 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[131 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][33 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[132 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][161 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[133 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][97 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[134 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][225 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[135 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][17 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[136 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][145 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[137 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][81 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[138 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][209 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[139 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][49 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[140 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][177 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[141 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][113 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[142 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][241 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[143 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][9 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[144 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][137 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[145 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][73 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[146 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][201 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[147 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][41 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[148 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][169 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[149 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][105 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[150 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][233 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[151 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][25 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[152 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][153 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[153 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][89 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[154 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][217 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[155 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][57 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[156 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][185 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[157 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][121 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[158 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][249 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[159 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][5 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[160 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][133 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[161 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][69 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[162 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][197 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[163 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][37 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[164 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][165 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[165 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][101 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[166 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][229 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[167 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][21 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[168 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][149 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[169 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][85 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[170 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][213 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[171 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][53 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[172 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][181 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[173 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][117 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[174 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][245 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[175 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][13 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[176 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][141 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[177 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][77 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[178 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][205 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[179 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][45 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[180 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][173 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[181 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][109 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[182 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][237 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[183 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][29 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[184 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][157 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[185 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][93 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[186 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][221 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[187 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][61 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[188 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][189 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[189 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][125 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[190 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][253 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[191 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[192 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][131 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[193 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][67 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[194 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][195 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[195 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][35 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[196 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][163 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[197 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][99 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[198 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][227 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[199 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][19 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[200 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][147 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[201 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][83 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[202 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][211 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[203 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][51 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[204 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][179 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[205 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][115 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[206 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][243 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[207 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][11 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[208 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][139 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[209 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][75 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[210 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][203 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[211 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][43 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[212 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][171 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[213 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][107 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[214 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][235 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[215 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][27 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[216 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][155 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[217 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][91 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[218 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][219 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[219 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][59 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[220 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][187 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[221 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][123 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[222 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][251 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[223 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][7 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[224 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][135 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[225 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][71 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[226 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][199 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[227 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][39 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[228 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][167 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[229 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][103 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[230 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][231 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[231 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][23 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[232 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][151 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[233 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][87 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[234 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][215 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[235 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][55 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[236 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][183 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[237 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][119 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[238 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][247 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[239 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][15 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[240 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][143 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[241 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][79 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[242 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][207 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[243 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][47 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[244 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][175 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[245 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][111 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[246 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][239 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[247 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][31 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[248 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][159 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[249 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][95 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[250 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][223 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[251 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][63 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[252 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][191 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[253 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][127 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[254 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][255 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[255 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_256VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 128) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][64 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][32 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][96 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][16 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[4 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][80 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[5 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][48 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[6 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][112 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[7 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][8 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[8 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][72 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[9 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][40 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[10 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][104 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[11 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][24 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[12 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][88 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[13 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][56 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[14 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][120 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[15 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][4 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[16 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][68 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[17 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][36 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[18 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][100 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[19 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][20 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[20 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][84 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[21 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][52 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[22 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][116 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[23 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][12 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[24 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][76 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[25 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][44 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[26 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][108 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[27 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][28 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[28 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][92 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[29 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][60 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[30 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][124 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[31 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[32 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][66 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[33 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][34 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[34 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][98 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[35 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][18 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[36 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][82 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[37 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][50 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[38 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][114 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[39 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][10 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[40 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][74 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[41 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][42 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[42 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][106 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[43 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][26 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[44 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][90 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[45 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][58 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[46 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][122 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[47 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][6 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[48 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][70 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[49 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][38 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[50 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][102 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[51 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][22 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[52 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][86 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[53 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][54 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[54 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][118 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[55 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][14 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[56 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][78 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[57 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][46 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[58 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][110 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[59 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][30 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[60 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][94 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[61 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][62 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[62 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][126 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[63 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[64 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][65 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[65 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][33 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[66 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][97 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[67 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][17 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[68 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][81 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[69 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][49 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[70 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][113 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[71 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][9 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[72 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][73 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[73 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][41 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[74 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][105 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[75 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][25 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[76 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][89 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[77 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][57 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[78 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][121 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[79 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][5 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[80 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][69 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[81 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][37 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[82 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][101 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[83 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][21 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[84 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][85 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[85 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][53 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[86 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][117 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[87 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][13 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[88 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][77 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[89 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][45 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[90 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][109 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[91 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][29 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[92 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][93 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[93 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][61 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[94 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][125 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[95 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[96 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][67 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[97 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][35 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[98 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][99 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[99 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][19 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[100 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][83 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[101 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][51 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[102 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][115 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[103 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][11 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[104 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][75 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[105 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][43 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[106 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][107 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[107 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][27 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[108 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][91 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[109 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][59 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[110 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][123 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[111 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][7 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[112 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][71 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[113 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][39 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[114 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][103 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[115 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][23 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[116 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][87 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[117 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][55 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[118 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][119 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[119 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][15 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[120 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][79 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[121 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][47 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[122 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][111 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[123 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][31 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[124 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][95 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[125 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][63 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[126 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][127 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[127 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_128VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 64) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][32 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][16 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][48 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][8 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[4 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][40 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[5 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][24 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[6 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][56 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[7 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][4 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[8 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][36 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[9 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][20 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[10 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][52 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[11 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][12 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[12 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][44 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[13 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][28 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[14 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][60 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[15 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[16 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][34 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[17 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][18 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[18 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][50 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[19 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][10 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[20 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][42 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[21 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][26 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[22 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][58 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[23 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][6 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[24 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][38 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[25 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][22 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[26 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][54 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[27 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][14 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[28 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][46 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[29 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][30 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[30 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][62 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[31 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[32 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][33 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[33 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][17 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[34 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][49 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[35 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][9 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[36 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][41 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[37 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][25 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[38 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][57 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[39 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][5 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[40 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][37 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[41 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][21 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[42 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][53 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[43 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][13 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[44 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][45 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[45 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][29 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[46 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][61 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[47 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[48 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][35 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[49 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][19 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[50 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][51 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[51 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][11 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[52 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][43 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[53 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][27 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[54 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][59 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[55 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][7 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[56 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][39 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[57 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][23 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[58 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][55 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[59 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][15 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[60 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][47 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[61 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][31 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[62 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][63 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[63 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_64VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 32) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][16 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][8 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][24 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][4 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[4 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][20 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[5 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][12 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[6 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][28 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[7 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[8 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][18 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[9 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][10 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[10 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][26 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[11 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][6 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[12 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][22 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[13 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][14 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[14 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][30 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[15 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[16 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][17 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[17 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][9 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[18 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][25 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[19 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][5 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[20 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][21 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[21 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][13 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[22 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][29 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[23 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[24 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][19 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[25 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][11 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[26 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][27 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[27 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][7 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[28 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][23 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[29 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][15 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[30 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][31 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[31 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_32VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 16) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][8 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][4 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][12 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[4 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][10 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[5 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][6 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[6 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][14 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[7 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[8 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][9 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[9 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][5 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[10 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][13 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[11 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[12 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][11 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[13 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][7 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[14 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][15 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[15 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_16VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 8) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][4 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][6 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[4 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][5 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[5 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[6 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][7 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[7 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_8VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 4) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][2 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[2 * BIT_WIDTH+:BIT_WIDTH];
			assign real_msg[0][3 * BIT_WIDTH+:BIT_WIDTH] = recv_msg[3 * BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_4VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
		else if (N_SAMPLES == 2) begin : genblk1
			assign real_msg[0][0+:BIT_WIDTH] = recv_msg[0+:BIT_WIDTH];
			assign real_msg[0][BIT_WIDTH+:BIT_WIDTH] = recv_msg[BIT_WIDTH+:BIT_WIDTH];
			SineWave__BIT_WIDTH_32__DECIMAL_POINT_16__SIZE_FFT_2VRTL SineWave(.sine_wave_out(sine_wave_out));
		end
	endgenerate
	genvar i;
	genvar b;
	generate
		for (i = 0; i < $clog2(N_SAMPLES); i = i + 1) begin : genblk2
			wire [N_SAMPLES * BIT_WIDTH:1] sv2v_tmp_fft_stage_send_msg_imag;
			always @(*) complex_msg[i + 1] = sv2v_tmp_fft_stage_send_msg_imag;
			FFT_StageVRTL #(
				.BIT_WIDTH(BIT_WIDTH),
				.DECIMAL_PT(DECIMAL_PT),
				.N_SAMPLES(N_SAMPLES),
				.STAGE_FFT(i)
			) fft_stage(
				.recv_msg_real(real_msg[i]),
				.recv_msg_imag(complex_msg[i]),
				.recv_val(val_in[i]),
				.recv_rdy(rdy_in[i]),
				.send_msg_real(real_msg[i + 1]),
				.send_msg_imag(sv2v_tmp_fft_stage_send_msg_imag),
				.send_val(val_in[i + 1]),
				.send_rdy(rdy_in[i + 1]),
				.sine_wave_out(sine_wave_out),
				.reset(reset),
				.clk(clk)
			);
		end
	endgenerate
	always @(*) begin : sv2v_autoblock_2
		reg signed [31:0] i;
		for (i = 0; i < N_SAMPLES; i = i + 1)
			send_msg[i * BIT_WIDTH+:BIT_WIDTH] = real_msg[$clog2(N_SAMPLES)][i * BIT_WIDTH+:BIT_WIDTH];
	end
endmodule
module SerializerVRTL (
	recv_msg,
	recv_val,
	recv_rdy,
	send_msg,
	send_val,
	send_rdy,
	reset,
	clk
);
	parameter BIT_WIDTH = 32;
	parameter N_SAMPLES = 8;
	input wire [(N_SAMPLES * BIT_WIDTH) - 1:0] recv_msg;
	input wire recv_val;
	output wire recv_rdy;
	output reg [BIT_WIDTH - 1:0] send_msg;
	output wire send_val;
	input wire send_rdy;
	input wire reset;
	input wire clk;
	wire [$clog2(N_SAMPLES) - 1:0] mux_sel;
	wire reg_en;
	wire [BIT_WIDTH - 1:0] reg_out [N_SAMPLES - 1:0];
	genvar i;
	generate
		for (i = 0; i < N_SAMPLES; i = i + 1) begin : genblk1
			vc_EnResetReg #(.p_nbits(BIT_WIDTH)) register(
				.clk(clk),
				.reset(reset),
				.en(reg_en),
				.d(recv_msg[i * BIT_WIDTH+:BIT_WIDTH]),
				.q(reg_out[i])
			);
		end
	endgenerate
	always @(*) send_msg = reg_out[mux_sel];
	SerializerControl #(.N_SAMPLES(N_SAMPLES)) ctrl(
		.clk(clk),
		.reset(reset),
		.recv_val(recv_val),
		.recv_rdy(recv_rdy),
		.send_val(send_val),
		.send_rdy(send_rdy),
		.mux_sel(mux_sel),
		.reg_en(reg_en)
	);
endmodule
module SerializerControl (
	recv_val,
	recv_rdy,
	send_val,
	send_rdy,
	mux_sel,
	reg_en,
	clk,
	reset
);
	parameter N_SAMPLES = 8;
	input wire recv_val;
	output reg recv_rdy;
	output reg send_val;
	input wire send_rdy;
	output reg [$clog2(N_SAMPLES) - 1:0] mux_sel;
	output reg reg_en;
	input wire clk;
	input wire reset;
	localparam INIT = 0;
	localparam OUTPUT_START = 1;
	localparam ADD = 2;
	reg next_state;
	reg state;
	reg [$clog2(N_SAMPLES):0] mux_sel_next;
	always @(*)
		case (state)
			INIT: begin
				if (reset == 1)
					next_state = INIT;
				if (recv_val == 1)
					next_state = OUTPUT_START;
				else
					next_state = INIT;
			end
			OUTPUT_START:
				if (mux_sel_next != N_SAMPLES)
					next_state = OUTPUT_START;
				else
					next_state = INIT;
			default: next_state = INIT;
		endcase
	always @(*)
		case (state)
			INIT: begin
				reg_en = 1;
				send_val = 0;
				recv_rdy = 1;
				mux_sel_next = 0;
			end
			OUTPUT_START: begin
				reg_en = 0;
				send_val = 1;
				recv_rdy = 0;
				if (send_rdy == 1)
					mux_sel_next = mux_sel + 1;
				else
					mux_sel_next = mux_sel;
			end
		endcase
	always @(posedge clk)
		if (reset)
			state <= INIT;
		else begin
			mux_sel <= mux_sel_next;
			state <= next_state;
		end
endmodule
module ControlVRTL (
	recv_val,
	send_rdy,
	send_val,
	recv_rdy,
	en_sel,
	reset,
	clk
);
	parameter N_SAMPLES = 8;
	input wire recv_val;
	input wire send_rdy;
	output reg send_val;
	output reg recv_rdy;
	output wire [N_SAMPLES - 1:0] en_sel;
	input wire reset;
	input wire clk;
	localparam [1:0] INIT = 2'b00;
	localparam [1:0] STATE1 = 2'b01;
	localparam [1:0] STATE2 = 2'b10;
	reg [$clog2(N_SAMPLES) + 1:0] count;
	reg [$clog2(N_SAMPLES) + 1:0] count_next;
	reg [1:0] next_state;
	reg [1:0] state;
	DecoderVRTL #(.BIT_WIDTH($clog2(N_SAMPLES))) decoder(
		.in(count),
		.out(en_sel)
	);
	always @(*)
		case (state)
			INIT:
				if (count_next == N_SAMPLES)
					next_state = STATE1;
				else
					next_state = INIT;
			STATE1:
				if (send_rdy == 1)
					next_state = INIT;
				else
					next_state = STATE1;
			default: next_state = INIT;
		endcase
	always @(*)
		case (state)
			INIT: begin
				if (recv_val == 1)
					count_next = count + 1;
				else
					count_next = count;
				recv_rdy = 1'b1;
				send_val = 1'b0;
			end
			STATE1: begin
				count_next = 0;
				recv_rdy = 1'b0;
				send_val = 1'b1;
			end
			default: begin
				count_next = 0;
				recv_rdy = 1'b1;
				send_val = 1'b0;
			end
		endcase
	always @(posedge clk)
		if (reset) begin
			count <= 0;
			state <= INIT;
		end
		else begin
			count <= count_next;
			state <= next_state;
		end
endmodule
module DecoderVRTL (
	in,
	out
);
	parameter BIT_WIDTH = 3;
	input wire [BIT_WIDTH - 1:0] in;
	output reg [(1 << BIT_WIDTH) - 1:0] out;
	always @(*) out = {{1 << (BIT_WIDTH - 1) {1'b0}}, 1'b1} << in;
endmodule
module DeserializerVRTL (
	recv_val,
	recv_rdy,
	recv_msg,
	send_val,
	send_rdy,
	send_msg,
	clk,
	reset
);
	parameter N_SAMPLES = 8;
	parameter BIT_WIDTH = 32;
	input wire recv_val;
	output wire recv_rdy;
	input wire [BIT_WIDTH - 1:0] recv_msg;
	output wire send_val;
	input wire send_rdy;
	output wire [(N_SAMPLES * BIT_WIDTH) - 1:0] send_msg;
	input wire clk;
	input wire reset;
	wire [N_SAMPLES - 1:0] en_sel;
	ControlVRTL #(.N_SAMPLES(N_SAMPLES)) c(
		.recv_val(recv_val),
		.send_rdy(send_rdy),
		.send_val(send_val),
		.recv_rdy(recv_rdy),
		.reset(reset),
		.clk(clk),
		.en_sel(en_sel)
	);
	genvar i;
	generate
		for (i = 0; i < N_SAMPLES; i = i + 1) begin : genblk1
			vc_EnResetReg #(.p_nbits(BIT_WIDTH)) register(
				.clk(clk),
				.reset(reset),
				.en(en_sel[i]),
				.d(recv_msg),
				.q(send_msg[i * BIT_WIDTH+:BIT_WIDTH])
			);
		end
	endgenerate
endmodule
module FFTSPIMinionVRTL (
	clk,
	reset,
	cs,
	sclk,
	mosi,
	miso,
	cs_2,
	sclk_2,
	mosi_2,
	miso_2
);
	parameter BIT_WIDTH = 32;
	parameter DECIMAL_PT = 16;
	parameter N_SAMPLES = 8;
	input wire clk;
	input wire reset;
	input wire cs;
	input wire sclk;
	input wire mosi;
	output wire miso;
	input wire cs_2;
	input wire sclk_2;
	input wire mosi_2;
	output wire miso_2;
	wire push_en_1;
	wire pull_en_1;
	wire [BIT_WIDTH + 1:0] push_msg_1;
	wire [BIT_WIDTH - 1:0] pull_msg_1;
	wire pull_msg_val_1;
	wire pull_msg_spc_1;
	wire push_en_2;
	wire pull_en_2;
	wire [BIT_WIDTH + 1:0] push_msg_2;
	wire [BIT_WIDTH - 1:0] pull_msg_2;
	wire pull_msg_val_2;
	wire pull_msg_spc_2;
	wire [BIT_WIDTH - 1:0] recv_msg_a_1;
	wire recv_rdy_a_1;
	wire recv_val_a_1;
	wire [BIT_WIDTH - 1:0] send_msg_a_2;
	wire send_rdy_a_2;
	wire send_val_a_2;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] recv_msg_s;
	wire recv_rdy_s;
	wire recv_val_s;
	wire [BIT_WIDTH - 1:0] send_msg_s;
	wire send_rdy_s;
	wire send_val_s;
	wire [BIT_WIDTH - 1:0] recv_msg_d;
	wire recv_rdy_d;
	wire recv_val_d;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] send_msg_d;
	wire send_rdy_d;
	wire send_val_d;
	wire minion1_parity;
	wire minion2_parity;
	wire adapter1_parity;
	wire adapter2_parity;
	SPI_minion_components_SPIMinionVRTL #(.nbits(BIT_WIDTH + 2)) minion1(
		.clk(clk),
		.cs(cs),
		.miso(miso),
		.mosi(mosi),
		.reset(reset),
		.sclk(sclk),
		.pull_en(pull_en_1),
		.pull_msg({pull_msg_val_1, pull_msg_spc_1, pull_msg_1}),
		.push_en(push_en_1),
		.push_msg(push_msg_1),
		.parity(minion1_parity)
	);
	SPI_minion_components_SPIMinionAdapterVRTL #(
		.nbits(BIT_WIDTH + 2),
		.num_entries(N_SAMPLES)
	) adapter1(
		.clk(clk),
		.reset(reset),
		.pull_en(pull_en_1),
		.pull_msg_val(pull_msg_val_1),
		.pull_msg_spc(pull_msg_spc_1),
		.pull_msg_data(pull_msg_1),
		.push_en(push_en_1),
		.push_msg_val_wrt(push_msg_1[BIT_WIDTH + 1]),
		.push_msg_val_rd(push_msg_1[BIT_WIDTH]),
		.push_msg_data(push_msg_1[BIT_WIDTH - 1:0]),
		.recv_msg(recv_msg_a_1),
		.recv_val(recv_val_a_1),
		.recv_rdy(recv_rdy_a_1),
		.send_msg(recv_msg_d),
		.send_val(recv_val_d),
		.send_rdy(recv_rdy_d),
		.parity(adapter1_parity)
	);
	DeserializerVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.N_SAMPLES(N_SAMPLES)
	) deserializer(
		.clk(clk),
		.reset(reset),
		.recv_msg(recv_msg_d),
		.recv_val(recv_val_d),
		.recv_rdy(recv_rdy_d),
		.send_msg(send_msg_d),
		.send_val(send_val_d),
		.send_rdy(send_rdy_d)
	);
	FFTVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.DECIMAL_PT(DECIMAL_PT),
		.N_SAMPLES(N_SAMPLES)
	) FFT(
		.clk(clk),
		.reset(reset),
		.recv_msg(send_msg_d),
		.recv_val(send_val_d),
		.recv_rdy(send_rdy_d),
		.send_msg(recv_msg_s),
		.send_val(recv_val_s),
		.send_rdy(recv_rdy_s)
	);
	SerializerVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.N_SAMPLES(N_SAMPLES)
	) serializer(
		.clk(clk),
		.reset(reset),
		.recv_msg(recv_msg_s),
		.recv_val(recv_val_s),
		.recv_rdy(recv_rdy_s),
		.send_msg(send_msg_s),
		.send_val(send_val_s),
		.send_rdy(send_rdy_s)
	);
	SPI_minion_components_SPIMinionAdapterVRTL #(
		.nbits(BIT_WIDTH + 2),
		.num_entries(N_SAMPLES)
	) adapter2(
		.clk(clk),
		.reset(reset),
		.pull_en(pull_en_2),
		.pull_msg_val(pull_msg_val_2),
		.pull_msg_spc(pull_msg_spc_2),
		.pull_msg_data(pull_msg_2),
		.push_en(push_en_2),
		.push_msg_val_wrt(push_msg_2[BIT_WIDTH + 1]),
		.push_msg_val_rd(push_msg_2[BIT_WIDTH]),
		.push_msg_data(push_msg_2[BIT_WIDTH - 1:0]),
		.recv_msg(send_msg_s),
		.recv_val(send_val_s),
		.recv_rdy(send_rdy_s),
		.send_msg(send_msg_a_2),
		.send_val(send_val_a_2),
		.send_rdy(send_rdy_a_2),
		.parity(adapter2_parity)
	);
	SPI_minion_components_SPIMinionVRTL #(.nbits(BIT_WIDTH + 2)) minion2(
		.clk(clk),
		.cs(cs_2),
		.miso(miso_2),
		.mosi(mosi_2),
		.reset(reset),
		.sclk(sclk_2),
		.pull_en(pull_en_2),
		.pull_msg({pull_msg_val_2, pull_msg_spc_2, pull_msg_2}),
		.push_en(push_en_2),
		.push_msg(push_msg_2),
		.parity(minion2_parity)
	);
	assign recv_val_a_1 = 0;
	assign recv_msg_a_1 = 0;
	assign send_rdy_a_2 = 0;
endmodule
module crossbarVRTL (
	recv_msg,
	recv_val,
	recv_rdy,
	send_msg,
	send_val,
	send_rdy,
	reset,
	clk,
	control,
	control_val,
	control_rdy
);
	parameter BIT_WIDTH = 32;
	parameter N_INPUTS = 2;
	parameter N_OUTPUTS = 2;
	parameter CONTROL_BIT_WIDTH = 42;
	input wire [(N_INPUTS * BIT_WIDTH) - 1:0] recv_msg;
	input wire [0:N_INPUTS - 1] recv_val;
	output reg [0:N_INPUTS - 1] recv_rdy;
	output reg [(N_OUTPUTS * BIT_WIDTH) - 1:0] send_msg;
	output reg [0:N_OUTPUTS - 1] send_val;
	input wire [0:N_OUTPUTS - 1] send_rdy;
	input wire reset;
	input wire clk;
	input wire [CONTROL_BIT_WIDTH - 1:0] control;
	input wire control_val;
	output wire control_rdy;
	reg [CONTROL_BIT_WIDTH - 1:0] stored_control;
	wire [$clog2(N_INPUTS) - 1:0] input_sel;
	wire [$clog2(N_OUTPUTS) - 1:0] output_sel;
	always @(posedge clk)
		if (reset)
			stored_control <= 0;
		else if (control_val)
			stored_control <= control;
	assign control_rdy = 1;
	assign input_sel = stored_control[CONTROL_BIT_WIDTH - 1:CONTROL_BIT_WIDTH - $clog2(N_INPUTS)];
	assign output_sel = stored_control[(CONTROL_BIT_WIDTH - $clog2(N_INPUTS)) - 1:(CONTROL_BIT_WIDTH - $clog2(N_INPUTS)) - $clog2(N_OUTPUTS)];
	always @(*) begin
		send_msg[((N_OUTPUTS - 1) - output_sel) * BIT_WIDTH+:BIT_WIDTH] = recv_msg[((N_INPUTS - 1) - input_sel) * BIT_WIDTH+:BIT_WIDTH];
		send_val[output_sel] = recv_val[input_sel];
		recv_rdy[input_sel] = send_rdy[output_sel];
		begin : sv2v_autoblock_1
			integer i;
			for (i = 0; i < N_OUTPUTS; i = i + 1)
				if (i != output_sel) begin
					send_msg[((N_OUTPUTS - 1) - i) * BIT_WIDTH+:BIT_WIDTH] = 0;
					send_val[i] = 0;
				end
		end
		begin : sv2v_autoblock_2
			integer i;
			for (i = 0; i < N_INPUTS; i = i + 1)
				if (i != input_sel)
					recv_rdy[i] = 0;
		end
	end
endmodule
module crossbaroneoutVRTL (
	recv_msg,
	recv_val,
	recv_rdy,
	send_msg,
	send_val,
	send_rdy,
	reset,
	clk,
	control,
	control_val,
	control_rdy
);
	parameter BIT_WIDTH = 32;
	parameter N_INPUTS = 2;
	parameter N_OUTPUTS = 1;
	parameter CONTROL_BIT_WIDTH = 32;
	input wire [(N_INPUTS * BIT_WIDTH) - 1:0] recv_msg;
	input wire [0:N_INPUTS - 1] recv_val;
	output reg [0:N_INPUTS - 1] recv_rdy;
	output reg [BIT_WIDTH - 1:0] send_msg;
	output reg send_val;
	input wire send_rdy;
	input wire reset;
	input wire clk;
	input wire [CONTROL_BIT_WIDTH - 1:0] control;
	input wire control_val;
	output wire control_rdy;
	reg [CONTROL_BIT_WIDTH - 1:0] stored_control;
	always @(posedge clk)
		if (reset)
			stored_control <= 0;
		else if (control_val)
			stored_control <= control;
	assign control_rdy = 1;
	wire [$clog2(N_INPUTS) - 1:0] input_sel;
	assign input_sel = stored_control[CONTROL_BIT_WIDTH - 1:CONTROL_BIT_WIDTH - $clog2(N_INPUTS)];
	always @(*) begin
		send_msg = recv_msg[((N_INPUTS - 1) - input_sel) * BIT_WIDTH+:BIT_WIDTH];
		send_val = recv_val[input_sel];
		recv_rdy[input_sel] = send_rdy;
		begin : sv2v_autoblock_1
			integer j;
			for (j = 0; j < N_INPUTS; j = j + 1)
				if (j != input_sel)
					recv_rdy[j] = 0;
		end
	end
endmodule
module parametricDemuxVRTL (
	in_val,
	sel,
	out_val
);
	parameter p_nbits = 1;
	parameter p_noutputs = 2;
	input wire [p_nbits - 1:0] in_val;
	input wire [$clog2(p_noutputs) - 1:0] sel;
	output wire [(p_noutputs * p_nbits) - 1:0] out_val;
	genvar i;
	generate
		for (i = 0; i < p_noutputs; i = i + 1) begin : output_gen
			assign out_val[i * p_nbits+:p_nbits] = (i == sel ? in_val : {p_nbits {1'b0}});
		end
	endgenerate
endmodule
module routerVRTL (
	valid,
	message_in,
	ready_out,
	valid_out,
	ready,
	message_out
);
	parameter p_nbits = 32;
	parameter p_noutputs = 8;
	input wire valid;
	input wire [p_nbits - 1:0] message_in;
	output wire ready_out;
	output wire [0:p_noutputs - 1] valid_out;
	input wire [0:p_noutputs - 1] ready;
	output wire [(p_noutputs * (p_nbits - $clog2(p_noutputs))) - 1:0] message_out;
	wire [p_noutputs - 1:0] valid_holder;
	wire [p_nbits - 1:p_nbits - $clog2(p_noutputs)] select;
	wire [(p_nbits - $clog2(p_noutputs)) - 1:0] cut_message;
	wire [p_noutputs - 1:0] temp_ready;
	assign select = message_in[p_nbits - 1:p_nbits - $clog2(p_noutputs)];
	assign cut_message = message_in[(p_nbits - $clog2(p_noutputs)) - 1:0];
	vc_MuxN #(
		.p_nbits(1),
		.p_ninputs(p_noutputs)
	) mux_inst(
		.in(temp_ready),
		.sel(select),
		.out(ready_out)
	);
	parametricDemuxVRTL #(
		.p_nbits(1),
		.p_noutputs(p_noutputs)
	) demux_inst(
		.in_val(valid),
		.sel(select),
		.out_val(valid_holder)
	);
	genvar i;
	generate
		for (i = 0; i < p_noutputs; i = i + 1) begin : genblk1
			assign valid_out[i] = valid_holder[i];
		end
	endgenerate
	genvar j;
	generate
		for (j = 0; j < p_noutputs; j = j + 1) begin : genblk2
			assign message_out[((p_noutputs - 1) - j) * (p_nbits - $clog2(p_noutputs))+:p_nbits - $clog2(p_noutputs)] = cut_message;
		end
	endgenerate
	genvar k;
	generate
		for (k = 0; k < p_noutputs; k = k + 1) begin : genblk3
			assign temp_ready[k+:1] = ready[k];
		end
	endgenerate
endmodule
module ArbitratorVRTL (
	clk,
	reset,
	recv_val,
	recv_rdy,
	recv_msg,
	send_val,
	send_rdy,
	send_msg
);
	parameter nbits = 32;
	parameter num_inputs = 3;
	parameter addr_nbits = $clog2(num_inputs);
	input wire clk;
	input wire reset;
	input wire [0:num_inputs - 1] recv_val;
	output reg [0:num_inputs - 1] recv_rdy;
	input wire [(num_inputs * nbits) - 1:0] recv_msg;
	output wire send_val;
	input wire send_rdy;
	output wire [(addr_nbits + nbits) - 1:0] send_msg;
	reg [addr_nbits - 1:0] grants_index;
	reg [addr_nbits - 1:0] old_grants_index;
	reg [addr_nbits - 1:0] encoder_out;
	wire [nbits - 1:0] send_msg_data;
	wire [addr_nbits - 1:0] send_msg_addr;
	assign send_msg_data = recv_msg[((num_inputs - 1) - grants_index) * nbits+:nbits];
	assign send_msg_addr = grants_index;
	assign send_val = recv_val[grants_index] & recv_rdy[grants_index];
	assign send_msg = {send_msg_addr, send_msg_data};
	always @(*)
		if (!recv_val[old_grants_index])
			grants_index = encoder_out;
		else
			grants_index = old_grants_index;
	always @(*) begin : sv2v_autoblock_1
		integer j;
		for (j = 0; j < num_inputs; j = j + 1)
			if (grants_index == j)
				recv_rdy[j] = send_rdy;
			else
				recv_rdy[j] = 1'b0;
	end
	always @(*) begin
		encoder_out = 0;
		begin : sv2v_autoblock_2
			integer i;
			for (i = 0; i < num_inputs; i = i + 1)
				if (recv_val[(num_inputs - 1) - i])
					encoder_out = (num_inputs - 1) - i;
		end
	end
	always @(posedge clk)
		if (reset)
			old_grants_index <= 0;
		else
			old_grants_index <= grants_index;
endmodule
module tape_in_FFT_interconnectVRTL (
	clk,
	reset,
	minion_cs,
	minion_sclk,
	minion_mosi,
	minion_miso,
	minion_parity,
	adapter_parity,
	minion_cs_2,
	minion_sclk_2,
	minion_mosi_2,
	minion_miso_2,
	minion_cs_3,
	minion_sclk_3,
	minion_mosi_3,
	minion_miso_3,
	master_cs,
	master_miso,
	master_mosi,
	master_sclk
);
	parameter BIT_WIDTH = 32;
	parameter DECIMAL_PT = 16;
	parameter N_SAMPLES = 8;
	input wire clk;
	input wire reset;
	input wire minion_cs;
	input wire minion_sclk;
	input wire minion_mosi;
	output wire minion_miso;
	output wire minion_parity;
	output wire adapter_parity;
	input wire minion_cs_2;
	input wire minion_sclk_2;
	input wire minion_mosi_2;
	output wire minion_miso_2;
	input wire minion_cs_3;
	input wire minion_sclk_3;
	input wire minion_mosi_3;
	output wire minion_miso_3;
	output wire master_cs;
	input wire master_miso;
	output wire master_mosi;
	output wire master_sclk;
	localparam MAX_ADDRESSABLE_SRCS_POW_2 = 16;
	localparam MAX_ADDRESSABLE_SRC_LOG2 = 4;
	wire spi_minion_send_val;
	wire spi_minion_send_rdy;
	wire [(BIT_WIDTH + MAX_ADDRESSABLE_SRC_LOG2) - 1:0] spi_minion_send_msg;
	wire spi_minion_recv_val;
	wire spi_minion_recv_rdy;
	wire [(BIT_WIDTH + MAX_ADDRESSABLE_SRC_LOG2) - 1:0] spi_minion_recv_msg;
	wire [0:15] module_interconnect_src_val;
	wire [0:15] module_interconnect_src_rdy;
	wire [(16 * BIT_WIDTH) - 1:0] module_interconnect_src_msg;
	wire [0:15] module_interconnect_snk_val;
	wire [0:15] module_interconnect_snk_rdy;
	wire [(16 * BIT_WIDTH) - 1:0] module_interconnect_snk_msg;
	wire spi_master_send_val;
	wire spi_master_send_rdy;
	wire [BIT_WIDTH - 1:0] spi_master_send_msg;
	wire spi_master_recv_val;
	wire spi_master_recv_rdy;
	wire [BIT_WIDTH - 1:0] spi_master_recv_msg;
	wire [(2 * BIT_WIDTH) - 1:0] fft_input_xbar_recv_msg;
	wire [0:1] fft_input_xbar_recv_val;
	wire [0:1] fft_input_xbar_recv_rdy;
	wire [(2 * BIT_WIDTH) - 1:0] fft_input_xbar_send_msg;
	wire [0:1] fft_input_xbar_send_val;
	wire [0:1] fft_input_xbar_send_rdy;
	wire [(2 * BIT_WIDTH) - 1:0] fft_output_xbar_recv_msg;
	wire [0:1] fft_output_xbar_recv_val;
	wire [0:1] fft_output_xbar_recv_rdy;
	wire [(2 * BIT_WIDTH) - 1:0] spi_master_xbar_recv_msg;
	wire [0:1] spi_master_xbar_recv_val;
	wire [0:1] spi_master_xbar_recv_rdy;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] recv_msg_s;
	wire recv_rdy_s;
	wire recv_val_s;
	wire [(N_SAMPLES * BIT_WIDTH) - 1:0] send_msg_d;
	wire send_rdy_d;
	wire send_val_d;
	wire [0:1] master_cs_temp;
	wire [(BIT_WIDTH + MAX_ADDRESSABLE_SRC_LOG2) - 1:0] arb_imm;
	wire deserializer_reset;
	genvar i;
	generate
		for (i = 10; i < MAX_ADDRESSABLE_SRCS_POW_2; i = i + 1) begin : genblk1
			assign module_interconnect_src_rdy[i] = 0;
			assign module_interconnect_snk_val[i] = 0;
			assign module_interconnect_snk_msg[(15 - i) * BIT_WIDTH+:BIT_WIDTH] = 0;
		end
	endgenerate
	SPIMinionAdapterConnectedVRTL #(
		.BIT_WIDTH(BIT_WIDTH + MAX_ADDRESSABLE_SRC_LOG2),
		.N_SAMPLES(N_SAMPLES)
	) ctrl_spi_minion(
		.clk(clk),
		.reset(reset),
		.cs(minion_cs),
		.sclk(minion_sclk),
		.mosi(minion_mosi),
		.miso(minion_miso),
		.recv_msg(spi_minion_recv_msg),
		.recv_rdy(spi_minion_recv_rdy),
		.recv_val(spi_minion_recv_val),
		.send_msg(spi_minion_send_msg),
		.send_rdy(spi_minion_send_rdy),
		.send_val(spi_minion_send_val),
		.minion_parity(minion_parity),
		.adapter_parity(adapter_parity)
	);
	ArbitratorVRTL #(
		.nbits(BIT_WIDTH),
		.num_inputs(MAX_ADDRESSABLE_SRCS_POW_2)
	) arbiter(
		.clk(clk),
		.reset(reset),
		.recv_val(module_interconnect_snk_val),
		.recv_rdy(module_interconnect_snk_rdy),
		.recv_msg(module_interconnect_snk_msg),
		.send_val(spi_minion_recv_val),
		.send_rdy(spi_minion_recv_rdy),
		.send_msg(spi_minion_recv_msg)
	);
	routerVRTL #(
		.p_nbits(BIT_WIDTH + MAX_ADDRESSABLE_SRC_LOG2),
		.p_noutputs(MAX_ADDRESSABLE_SRCS_POW_2)
	) router(
		.valid(spi_minion_send_val),
		.ready_out(spi_minion_send_rdy),
		.message_in(spi_minion_send_msg),
		.valid_out(module_interconnect_src_val),
		.ready(module_interconnect_src_rdy),
		.message_out(module_interconnect_src_msg)
	);
	assign module_interconnect_snk_val[0] = module_interconnect_src_val[0];
	assign module_interconnect_src_rdy[0] = module_interconnect_snk_rdy[0];
	assign module_interconnect_snk_msg[15 * BIT_WIDTH+:BIT_WIDTH] = module_interconnect_src_msg[15 * BIT_WIDTH+:BIT_WIDTH];
	assign fft_input_xbar_recv_msg[BIT_WIDTH+:BIT_WIDTH] = module_interconnect_src_msg[8 * BIT_WIDTH+:BIT_WIDTH];
	assign fft_input_xbar_recv_val[0] = module_interconnect_src_val[7];
	assign module_interconnect_src_rdy[7] = fft_input_xbar_recv_rdy[0];
	assign fft_input_xbar_recv_msg[0+:BIT_WIDTH] = spi_master_send_msg;
	assign fft_input_xbar_recv_val[1] = spi_master_send_val;
	assign spi_master_send_rdy = fft_input_xbar_recv_rdy[1];
	crossbarVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.N_INPUTS(2),
		.N_OUTPUTS(2),
		.CONTROL_BIT_WIDTH(BIT_WIDTH)
	) fft_input_xbar(
		.clk(clk),
		.reset(reset),
		.recv_msg(fft_input_xbar_recv_msg),
		.recv_val(fft_input_xbar_recv_val),
		.recv_rdy(fft_input_xbar_recv_rdy),
		.send_msg(fft_input_xbar_send_msg),
		.send_val(fft_input_xbar_send_val),
		.send_rdy(fft_input_xbar_send_rdy),
		.control(module_interconnect_src_msg[14 * BIT_WIDTH+:BIT_WIDTH]),
		.control_val(module_interconnect_src_val[1]),
		.control_rdy(module_interconnect_src_rdy[1])
	);
	assign fft_output_xbar_recv_msg[0+:BIT_WIDTH] = fft_input_xbar_send_msg[0+:BIT_WIDTH];
	assign fft_output_xbar_recv_val[1] = fft_input_xbar_send_val[1];
	assign fft_input_xbar_send_rdy[1] = fft_output_xbar_recv_rdy[1];
	crossbaroneoutVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.N_INPUTS(2),
		.N_OUTPUTS(1),
		.CONTROL_BIT_WIDTH(BIT_WIDTH)
	) fft_output_xbar(
		.clk(clk),
		.reset(reset),
		.recv_msg(fft_output_xbar_recv_msg),
		.recv_val(fft_output_xbar_recv_val),
		.recv_rdy(fft_output_xbar_recv_rdy),
		.send_msg(module_interconnect_snk_msg[14 * BIT_WIDTH+:BIT_WIDTH]),
		.send_val(module_interconnect_snk_val[1]),
		.send_rdy(module_interconnect_snk_rdy[1]),
		.control(module_interconnect_src_msg[13 * BIT_WIDTH+:BIT_WIDTH]),
		.control_val(module_interconnect_src_val[2]),
		.control_rdy(module_interconnect_src_rdy[2])
	);
	crossbaroneoutVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.N_INPUTS(2),
		.N_OUTPUTS(1),
		.CONTROL_BIT_WIDTH(BIT_WIDTH)
	) spi_master_xbar(
		.clk(clk),
		.reset(reset),
		.recv_msg(spi_master_xbar_recv_msg),
		.recv_val(spi_master_xbar_recv_val),
		.recv_rdy(spi_master_xbar_recv_rdy),
		.send_msg(spi_master_recv_msg),
		.send_val(spi_master_recv_val),
		.send_rdy(spi_master_recv_rdy),
		.control(module_interconnect_src_msg[9 * BIT_WIDTH+:BIT_WIDTH]),
		.control_val(module_interconnect_src_val[6]),
		.control_rdy(module_interconnect_src_rdy[6])
	);
	assign spi_master_xbar_recv_msg[BIT_WIDTH+:BIT_WIDTH] = module_interconnect_src_msg[7 * BIT_WIDTH+:BIT_WIDTH];
	assign spi_master_xbar_recv_val[0] = module_interconnect_src_val[8];
	assign module_interconnect_src_rdy[8] = spi_master_xbar_recv_rdy[0];
	assign spi_master_xbar_recv_msg[0+:BIT_WIDTH] = 0;
	assign spi_master_xbar_recv_val[1] = 1;
	assign master_cs = master_cs_temp[0];
	SPIMasterValRdyVRTL #(
		.nbits(32),
		.ncs(2)
	) spi_master(
		.clk(clk),
		.reset(reset),
		.spi_ifc_cs(master_cs_temp),
		.spi_ifc_miso(master_miso),
		.spi_ifc_mosi(master_mosi),
		.spi_ifc_sclk(master_sclk),
		.recv_val(spi_master_recv_val),
		.recv_rdy(spi_master_recv_rdy),
		.recv_msg(spi_master_recv_msg),
		.send_val(spi_master_send_val),
		.send_rdy(spi_master_send_rdy),
		.send_msg(spi_master_send_msg),
		.packet_size_ifc_val(module_interconnect_src_val[5]),
		.packet_size_ifc_rdy(module_interconnect_src_rdy[5]),
		.packet_size_ifc_msg({1'b0, module_interconnect_src_msg[(10 * BIT_WIDTH) + ((BIT_WIDTH - 1) >= (BIT_WIDTH - 6) ? BIT_WIDTH - 1 : ((BIT_WIDTH - 1) + ((BIT_WIDTH - 1) >= (BIT_WIDTH - 6) ? ((BIT_WIDTH - 1) - (BIT_WIDTH - 6)) + 1 : ((BIT_WIDTH - 6) - (BIT_WIDTH - 1)) + 1)) - 1)-:((BIT_WIDTH - 1) >= (BIT_WIDTH - 6) ? ((BIT_WIDTH - 1) - (BIT_WIDTH - 6)) + 1 : ((BIT_WIDTH - 6) - (BIT_WIDTH - 1)) + 1)]}),
		.cs_addr_ifc_val(module_interconnect_src_val[4]),
		.cs_addr_ifc_rdy(module_interconnect_src_rdy[4]),
		.cs_addr_ifc_msg(module_interconnect_src_msg[(11 * BIT_WIDTH) + ((BIT_WIDTH - 1) >= (BIT_WIDTH - 2) ? BIT_WIDTH - 1 : ((BIT_WIDTH - 1) + ((BIT_WIDTH - 1) >= (BIT_WIDTH - 2) ? ((BIT_WIDTH - 1) - (BIT_WIDTH - 2)) + 1 : ((BIT_WIDTH - 2) - (BIT_WIDTH - 1)) + 1)) - 1)-:((BIT_WIDTH - 1) >= (BIT_WIDTH - 2) ? ((BIT_WIDTH - 1) - (BIT_WIDTH - 2)) + 1 : ((BIT_WIDTH - 2) - (BIT_WIDTH - 1)) + 1)]),
		.freq_ifc_val(module_interconnect_src_val[3]),
		.freq_ifc_rdy(module_interconnect_src_rdy[3]),
		.freq_ifc_msg(module_interconnect_src_msg[(12 * BIT_WIDTH) + ((BIT_WIDTH - 1) >= (BIT_WIDTH - 3) ? BIT_WIDTH - 1 : ((BIT_WIDTH - 1) + ((BIT_WIDTH - 1) >= (BIT_WIDTH - 3) ? ((BIT_WIDTH - 1) - (BIT_WIDTH - 3)) + 1 : ((BIT_WIDTH - 3) - (BIT_WIDTH - 1)) + 1)) - 1)-:((BIT_WIDTH - 1) >= (BIT_WIDTH - 3) ? ((BIT_WIDTH - 1) - (BIT_WIDTH - 3)) + 1 : ((BIT_WIDTH - 3) - (BIT_WIDTH - 1)) + 1)])
	);
	assign deserializer_reset = reset || module_interconnect_src_val[9];
	assign module_interconnect_src_rdy[9] = 1;
	DeserializerVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.N_SAMPLES(N_SAMPLES)
	) deserializer(
		.clk(clk),
		.reset(deserializer_reset),
		.recv_msg(fft_input_xbar_send_msg[BIT_WIDTH+:BIT_WIDTH]),
		.recv_val(fft_input_xbar_send_val[0]),
		.recv_rdy(fft_input_xbar_send_rdy[0]),
		.send_msg(send_msg_d),
		.send_val(send_val_d),
		.send_rdy(send_rdy_d)
	);
	FFTVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.DECIMAL_PT(DECIMAL_PT),
		.N_SAMPLES(N_SAMPLES)
	) FFT(
		.clk(clk),
		.reset(reset),
		.recv_msg(send_msg_d),
		.recv_val(send_val_d),
		.recv_rdy(send_rdy_d),
		.send_msg(recv_msg_s),
		.send_val(recv_val_s),
		.send_rdy(recv_rdy_s)
	);
	SerializerVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.N_SAMPLES(N_SAMPLES)
	) serializer(
		.clk(clk),
		.reset(reset),
		.recv_msg(recv_msg_s),
		.recv_val(recv_val_s),
		.recv_rdy(recv_rdy_s),
		.send_msg(fft_output_xbar_recv_msg[BIT_WIDTH+:BIT_WIDTH]),
		.send_val(fft_output_xbar_recv_val[0]),
		.send_rdy(fft_output_xbar_recv_rdy[0])
	);
	FFTSPIMinionVRTL #(
		.BIT_WIDTH(BIT_WIDTH),
		.DECIMAL_PT(DECIMAL_PT),
		.N_SAMPLES(8)
	) tape_in_one_model(
		.clk(clk),
		.reset(reset),
		.cs(minion_cs_2),
		.sclk(minion_sclk_2),
		.mosi(minion_mosi_2),
		.miso(minion_miso_2),
		.cs_2(minion_cs_3),
		.sclk_2(minion_sclk_3),
		.mosi_2(minion_mosi_3),
		.miso_2(minion_miso_3)
	);
endmodule
module FFTSPIInterconnectRTL (
	adapter_parity,
	clk,
	minion_parity,
	reset,
	minion_cs,
	minion_cs_2,
	minion_cs_3,
	minion_miso,
	minion_miso_2,
	minion_miso_3,
	minion_mosi,
	minion_mosi_2,
	minion_mosi_3,
	master_cs,
	master_miso,
	master_mosi,
	master_sclk,
	minion_sclk,
	minion_sclk_2,
	minion_sclk_3,
	io_oeb,
  	`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1	// User area 1 digital ground
	`endif
	
);
	output wire [0:0] adapter_parity;
	input wire [0:0] clk;
	output wire [0:0] minion_parity;
	input wire [0:0] reset;
	input wire [0:0] minion_cs;
	input wire [0:0] minion_cs_2;
	input wire [0:0] minion_cs_3;
	output wire [0:0] minion_miso;
	output wire [0:0] minion_miso_2;
	output wire [0:0] minion_miso_3;
	input wire [0:0] minion_mosi;
	input wire [0:0] minion_mosi_2;
	input wire [0:0] minion_mosi_3;
	output wire [0:0] master_cs;
	input wire [0:0] master_miso;
	output wire [0:0] master_mosi;
	output wire [0:0] master_sclk;
	input wire [0:0] minion_sclk;
	input wire [0:0] minion_sclk_2;
	input wire [0:0] minion_sclk_3;
	output wire [18:0] io_oeb;

	assign io_oeb = 19'b100001110111011100;
	tape_in_FFT_interconnectVRTL #(
		.BIT_WIDTH(32),
		.DECIMAL_PT(16),
		.N_SAMPLES(4)
	) v(
		.adapter_parity(adapter_parity),
		.clk(clk),
		.minion_parity(minion_parity),
		.reset(reset),
		.minion_cs(minion_cs),
		.minion_cs_2(minion_cs_2),
		.minion_cs_3(minion_cs_3),
		.minion_miso(minion_miso),
		.minion_miso_2(minion_miso_2),
		.minion_miso_3(minion_miso_3),
		.minion_mosi(minion_mosi),
		.minion_mosi_2(minion_mosi_2),
		.minion_mosi_3(minion_mosi_3),
		.master_cs(master_cs),
		.master_miso(master_miso),
		.master_mosi(master_mosi),
		.master_sclk(master_sclk),
		.minion_sclk(minion_sclk),
		.minion_sclk_2(minion_sclk_2),
		.minion_sclk_3(minion_sclk_3)
	);
endmodule
