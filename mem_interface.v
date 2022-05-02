`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:30:03 04/30/2022 
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
	hw_ram_rasn, hw_ram_casn, hw_ram_wen, hw_ram_ba, hw_ram_udqs_p, 
	hw_ram_udqs_n, hw_ram_ldqs_p, hw_ram_ldqs_n, hw_ram_udm, hw_ram_ldm, hw_ram_ck, 
	hw_ram_ckn, hw_ram_cke, hw_ram_odt, hw_ram_ad, hw_ram_dq, hw_rzq_pin, hw_zio_pin,
	CLK, reset, read_out, addr_in, data_in, status, pb_state);
	
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
	
	input 	CLK, reset;
	output 	status;
	input pb_state;
	
	// REMOVED: address, data, and read data will be handled from picoBlaze CPU
	/*
	input  [7:0]	switches; 		// address
	input	 [7:0]	dip_switches; 	// data to be written into RAM
	output [7:0]	leds;			// data read out of RAM
	*/
	
	input  [25:0] addr_in;	// address
	input  [7:0]  data_in;	// data to be written into RAM
	output [7:0]  read_out;	// data read out of RAM
	
	reg 	[25:0] address = 0;
	reg 	[7:0]	RAMin;
	wire 	[7:0]	RAMout;
	reg	[7:0] dataOut; 
	reg 			reqRead;
	reg 			enableWrite;
	reg 			ackRead = 0;
	wire 			systemCLK;
	
	wire rdy, 	dataPresent;
	wire [25:0]	max_ram_address;
	
	reg [3:0]	state=4'b0000;
	
	parameter stInit = 4'b0000;
	parameter stReadFromPorts = 4'b0001;
	parameter stMemWrite = 4'b0010;
	parameter stMemReadReq  = 4'b0011;
	parameter stMemReadData = 4'b0100;
	
	// FSM to read and write to DDR2 RAM
	always @(posedge systemCLK)
	begin
		if (reset) begin 
			address <= 0;
			state <= stInit;
		end
		else
			case (pb_state)
				// PLAY state
				8'h00: begin
					state <= stMemReadReq;
				end
				// RECORD State
				8'h01: begin
					state <= stReadFromPorts;
				end
				// DELETE STATE
				8'h02: begin
					state <= stReadFromPorts;
			if(rdy) begin // Only if RAM is rdy for read/write
				
				case (state)

				  // Initialization state
				  stInit: begin 
				   ackRead <= 1'b0;
					state <= stReadFromPorts;
				  end
				  
				  // Read from the ports
				  // switches are read and used as address
				  // dip_switches are read and used as data to be written into RAM
				  stReadFromPorts: begin
				   address <= addr_in;
				   RAMin <= data_in;
					state <= stMemWrite;
				  end
				  
				  // Write cycle, raise write enable
				  stMemWrite: begin
				   enableWrite <= 1'b1;
					state <= stMemReadReq;
				  end
				  
				  // Read cycle 1, pull down write enable, raise read request
				  stMemReadReq: begin
					enableWrite <= 1'b0;
					reqRead <= 1'b1;
					state <= stMemReadData;
				  end
				  
				  // Read cycle 2
				  // Waite until data is valid i.e., when dataPresent is 1
				  stMemReadData: begin
					reqRead <= 1'b0;
					if(dataPresent) begin // data is present, read to dataOut register
						dataOut = RAMout;
						ackRead <= 1'b1;	 // acknowledge the read
						state <= stInit;
					end
					else begin				  // stay in the same state until data is valid
						state <= stMemReadData;
					end
				  end
			 
			 endcase 
			end // rdy
		end
	
	// ram interface instantiation
	ram_interface_wrapper RAMRapper (
					.address(address),				// input 
					.data_in(RAMin), 					// input
					.write_enable(enableWrite), 	//	input
					.read_request(reqRead), 		//	input
					.read_ack(ackRead), 
					.data_out(RAMout), 				// output from ram to wire
					.reset(reset), 
					.clk(CLK), 
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
					.clkout(systemCLK), 
					.sys_clk(systemCLK), 
					.rdy(rdy), 
					.rd_data_pres(dataPresent),
					.max_ram_address(max_ram_address)
					);		
					
assign status = rdy; // ready signal from ram
assign read_out = dataOut; // output data read to leds
		
endmodule
