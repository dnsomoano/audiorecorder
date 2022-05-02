`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    01:40:15 04/30/2022 
// Design Name: 
// Module Name:    hex_to_bin 
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
module hex_to_bin(
    d_input,
    d_out,
    );

input [15:0] d_input;
output [31:0] d_out = 0;

reg [1:0] index = 2'd0;
reg o_array [0:15];
reg dec_output = 0;

parameter A = 2'd10;
parameter B = 2'd11;
parameter C = 2'd12;
parameter D = 2'd13;
parameter E = 2'd14;
parameter F = 2'd15;

always @(posedge clk) begin
    while (index <= 2'd15) begin
        // TODO easier approach each hex digit converts to 4-bit binary digit.
        assign o_array[index] = d_input[index] * (16 ** index);
        assign index = index + 1;
    end
    // assign index = 0;
    // while (index <= 2'd15) begin
    //     assign dec_output = dec_output + o_array[index];
    //     assign index = index + 1;
    // end
    // * Convert decimal to binary value
    // * Euclide
    d_out = de
end

endmodule
