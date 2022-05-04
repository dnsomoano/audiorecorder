`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:00:36 05/02/2022 
// Design Name: 
// Module Name:    audio_recorder 
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
module audio_recorder(
	input  clk,
	input  rst,
	input  en,
	input  aud_rdy,
	input  init,
	input  [2:0] track_sel,
	input  [25:0] max_mem_addr,
	output track_full,
	output [25:0] address,
	output write_en
);

reg [25:0] address;
reg track_full, write_en;

// Wires/Registers
reg [25:0] max_address = 0;
	
// State register
reg state = 1'b0;

// Audio recording logic
always@ (posedge clk) begin
	// Synchronous reset
	if (reset) begin
		address 		<= 0;
		max_address <= 0;
		track_full  <= 1'b0;
		write			<= 1'b0;
	end
	// Memory logic
	else if (en) begin
		if (init) begin		// Initialize addresses based on selected track
			case (track_id)
				3'b001: begin
					address 		<= 0;
					max_address <= 26'h333332;
				end
				3'b010: begin
					address		<= 26'h333333;
					max_address	<= 26'h666665;
					
				end
				3'b011: begin
					address		<= 26'h666666;
					max_address	<= 26'h999998;
				end
				3'b100: begin
					address		<= 26'h999999;
					max_address	<= 26'hCCCCCB;
				end
				3'b101: begin
					address		<= 26'hCCCCCC;
					max_address <= max_mem_addr;
				end
			endcase
		end
	end
	// Memory writing logic
	else if (address < max_address) begin
		track_full <= 1'b0;
		case (state)
			// Increase memory address bit-by-bit if audio data is present
			1'b0: begin
				if (aud_rdy) begin
					address	<= address + 1'b1;
					write_en <= 1'b1;
					state		<= 1'b1;
				end
			end
			1'b1: begin
				write_en <= 1'b0;
				state		<= 1'b0;
			end
		endcase
	end
	// Stop if memory is full for current track
	else if (address == max_address)
		track_full <= 1'b1;
end

endmodule
