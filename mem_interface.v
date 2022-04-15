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

input [25:0] address;
input [25:0] data_in;
input write_enable;
input read_request;
input read_ack;
input reset, clk, sys_clk; 

ram_interface_wrapper mem_wrapper{
    .address(address),
    .data_in(data_in),
    .write_enable(write_enable),
    .read_request(read_request),
    .read_ack(read_ack),
    .reset(reset),
    .clk(clk),
    .sys_clk(sys_clk),
    .max
}

endmodule
