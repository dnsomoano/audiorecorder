`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    16:02:10 04/24/2022 
// Design Name: 
// Module Name:    audiomessagerecorder 
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
module audiomessagerecorder( reset, clk, switches, leds, rs232_rx, rs232_tx, button, data_in, write_enable, read_request, read_ack, data_out, reset, clk, hw_ram_rasn, hw_ram_casn,
	hw_ram_wen, hw_ram_ba, hw_ram_udqs_p, hw_ram_udqs_n, hw_ram_ldqs_p, hw_ram_ldqs_n, hw_ram_udm, hw_ram_ldm, hw_ram_ck, hw_ram_ckn, hw_ram_cke, hw_ram_odt,
	hw_ram_ad, hw_ram_dq, hw_rzq_pin, hw_zio_pin, clkout, sys_clk, rdy, rd_data_pres,
	max_ram_address, ledRAM, sample_end);
		
   
   //Inputs and Outputs
	input			reset;	
	input			clk;
	
	input	[7:0]	switches;
	output	[7:0]	leds;
	
	input		rs232_rx;
	output	rs232_tx;
	input    [3:0] button;
	
	//check
	input    data_in;
	output   data_out;
	input    write_enable;
	output   clkout;
	input    rdy;
	input    max_ram_address;
	input    read_request;
	input    sys_clk;
	input    read_ack;
	input    rd_data_pres;
	input    sample_end;
	
	
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
	output ledRAM;
	
	
	// Wires and Register Declarations
	// PicoBlaze Data Lines
	wire	[7:0]	pb_port_id;
	wire	[7:0]	pb_out_port;
	reg		[7:0]	pb_in_port;
	wire			pb_read_strobe;
	wire			pb_write_strobe;
	
	// PicoBlaze Wires
	wire			pb_reset;
	wire			pb_interrupt;
	wire			pb_int_ack;
	
	// UART wires
	wire			write_to_uart;
	wire			uart_buffer_full;
	wire			uart_data_present;
	reg			read_from_uart;
	wire			uart_reset;
	
	// UART Data Lines
	wire	[7:0]	uart_rx_data;
	
	// LED wires
	wire write_to_leds;
	wire led_reset;

	wire main_clk;
	wire ram_clk;
	wire audio_clk;
	
	
	wire [1:0] sample_end;
	wire [1:0] sample_req;
	wire [15:0] audio_input;



	//Other registers
	reg [1:0] recordmode = 2'b00;
	reg [15:0] audio_output;

   reg [15:0] DDR2_data_in;
	reg [25:0] DDR2_address = 26'h0000000; 
	reg [25:0] DDR2_max_address;		 
	reg DDR2_write_enable = 1'b1;
	
	
	wire systemCLK;
	wire DDR2_rdy;
	wire DDR2_rd_data_pres;
		
	reg DDR2_read_request;
	reg DDR2_read_ack;
	wire [15:0] DDR2_data_out;

	reg [7:0] DDR2_address_low;
	reg [7:0] DDR2_address_mid;
	reg [7:0] DDR2_address_high;
	reg [2:0] DDR2_address_banksel;


	reg 	[25:0] address = 0;
	reg 	[15:0]	RAMin;
	wire 	[15:0]	RAMout;
	reg	[7:0] dataOut; 
	reg 			reqRead;
	reg 			enableWrite;
	reg 			ackRead = 0;

	reg    [7:0] audio_ramhi;
	reg    [7:0] audio_ramlo;
	reg freqcount;
	wire rdy, 	dataPresent;
	wire [25:0]	max_ram_address;
	
	reg [3:0]	state=4'b0000;
	
	parameter stInit = 4'b0000;
	parameter stReadFromPorts = 3'b001;
	parameter stMemWrite = 3'b0010;
	parameter stMemReadReq  = 3'b0011;
	parameter stMemReadData = 3'b0100;
	
//reg [15:0] last_sample;
reg [15:0] audio_input_sample;
reg [15:0] audio_output_sample;
reg [15:0] debug_sample;
reg ram_write_request;
reg ram_read_request;
reg ram_write_ack;
reg ram_read_ack;
	

	// FSM to read and write to DDR2 RAM
	always @(posedge systemCLK)
	begin
		if (reset) begin 
			address <= 0;
			state <= stInit;
		end
		else
			if(rdy) begin // Only if RAM is rdy for read/write
				
				case (state)

				  // Initialization state
				  stInit: begin 
				   ackRead <= 1'b0;
					enableWrite <= 1'b0;
					if(switches[2]) begin  //reset address to 0
						address <= 0;
					end
					else begin
						address <= address;
					end
					
					if(!switches[0] & switches[1] & !switches[2]) begin //record
						if (address == max_ram_address) begin
							freqcount <= 1'b1;
						end
						else begin
							address <= address + 1'b1;
							freqcount <= 1'b0;
						end
						state <= stReadFromPorts;
					end
					else if (switches[0] & switches[1] & !switches[2]) begin //playback
						if (address == max_ram_address) begin
							freqcount <= 1'b1;
						end
						else begin
							address <= address + 1'b1;
							freqcount <= 1'b0;
						end
						state <= stMemReadReq;
					end
					else begin
						state <= stInit;
					end
					end
				  
				  // Read from the ports where the switches will be used as address to read
				  // dip_switches are read and used as data to be written into RAM
				  stReadFromPorts: begin
					if(sample_end) begin
					RAMin <= audio_input_sample;
					state <= stMemWrite;
					end
					else begin
						state <= stReadFromPorts;
					end
				  end
				  
				  // Write cycle, raise write enable
				  stMemWrite: begin
				   enableWrite <= 1'b1;
					state <= stInit;
				  end
				  
				  // Read cycle 1, pull down write enable, raise read request
				  stMemReadReq: begin
				  if(sample_end) begin
					enableWrite <= 1'b0;
					reqRead <= 1'b1;
					state <= stMemReadData;
				  end
				  else begin
						state <= stMemReadReq;
					end
				end
				  
				  // Read cycle 2
				  // Waite until data is valid i.e., when dataPresent is 1
				  stMemReadData: begin
					reqRead <= 1'b0;
					if(dataPresent) begin // data is present, read to dataOut register
						audio_output_sample = RAMout;
						ackRead <= 1'b1;	 // acknowledge the reading of the data
						state <= stInit;
					end
					else begin				  // stay in the same state until data is valid
						state <= stMemReadData;
					end
				  end
			 
			 endcase 
			end 
		end

always @(posedge audio_clk) begin
    if (sample_end) begin
        audio_input_sample <= audio_input;
    end
end

endmodule
