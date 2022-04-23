`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    15:19:34 04/14/2022 
// Design Name: 
// Module Name:    mem_interface 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module mem_interface(
    address, 
    data_in, 
    write_enable, 
    read_request,
    read_ack,
    reset,
    clk,
    sys_clk
    );

output 	hw_ram_rasn;
output 	hw_ram_casn;
output 	hw_ram_wen;
output[2:0] hw_ram_ba;
inout 	hw_ram_udqs_p;
inout 	hw_ram_udqs_n;
inout 	hw_ram_ldqs_p;
inout 	hw_ram_ldqs_n;
output 	hw_ram_udm;
output 	hw_ram_ldm;
output 	hw_ram_ck;
output 	hw_ram_ckn;
output 	hw_ram_cke;
output 	hw_ram_odt;
output[12:0] hw_ram_ad;
inout [15:0] hw_ram_dq;
inout 	hw_rzq_pin;
inout 	hw_zio_pin;

input reset, clk, sys_clk; 
output 	status;

output [7:0]	leds;			// data read out of RAM

reg [25:0] address = 0;
//input [25:0] data_in;
reg [7:0] RAMin;
wire [7:0] RAMout;
reg	[7:0] dataOut; 
reg writeEnable;
reg read_request;
reg read_ack = 0;

wire [25:0]	max_ram_address;
wire rdy;


ram_interface_wrapper mem_wrapper(
    .address(address),
    .data_in(RAMin),
    .write_enable(writeEnable),
    .read_request(read_request),
    .read_ack(read_ack),
    .data_out(RAMout),
    .reset(reset),
    .clk(clk),
	.hw_ram_rasn(hw_ram_rasn), 
	.hw_ram_casn(hw_ram_casn),
	.hw_ram_wen(hw_ram_wen), 
	.hw_ram_ba(hw_ram_ba), 
	.hw_ram_udqs_p(hw_ram_udqs_p), 
	.hw_ram_udqs_n(hw_ram_udqs_n), 
	.hw_ram_ldqs_p(hw_ram_ldqs_p), 
	.hw_ram_ldqs_n(hw_ram_ldqs_n), 
	.hw_ram_udm(hw_ram_udm), 
	.hw_ram_ldm(hw_ram_ldm), 
	.hw_ram_ck(hw_ram_ck), 
	.hw_ram_ckn(hw_ram_ckn), 
	.hw_ram_cke(hw_ram_cke), 
	.hw_ram_odt(hw_ram_odt),
	.hw_ram_ad(hw_ram_ad), 
	.hw_ram_dq(hw_ram_dq), 
	.hw_rzq_pin(hw_rzq_pin), 
	.hw_zio_pin(hw_zio_pin), 
	.clkout(sys_clk), 
    .sys_clk(sys_clk),
	.rdy(rdy), 
	.rd_data_pres(dataPresent),
	.max_ram_address(max_ram_address)
);

assign status = rdy; // ready signal from ram
assign leds = dataOut; // output data read to leds

endmodule
