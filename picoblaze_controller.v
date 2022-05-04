`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:47:10 04/27/2022 
// Design Name: 
// Module Name:    picoblaze_controller 
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
module picoblaze_controller(
	// Control - clk, rst, buttons
	input  CLK,					// CLOCK 100MHz
	input	 RST,					// Reset: ACTIVE LOW!!!!!!
	input  [3:0] KYPD_COL,
	input	 [3:0] KYPD_ROW,
	output [3:0] leds,
	
	// Audio Interface IO
	inout  AUD_ADCLRCK,
   input  AUD_ADCDAT,
   inout  AUD_DACLRCK,
   output AUD_DACDAT,
   output AUD_XCK,
   inout  AUD_BCLK,
   output AUD_I2C_SCLK,
   inout  AUD_I2C_SDAT,
   output AUD_MUTE,
	output PLL_LOCKED,
	
	// RAM Interface IO
	output HW_RAM_RASN,
	output HW_RAM_CASN,
	output HW_RAM_WEN,
	output [2:0] HW_RAM_BA,
	inout  HW_RAM_UDQS_P,
	inout  HW_RAM_UDQS_N,
	inout  HW_RAM_LDQS_P,
	inout  HW_RAM_LDQS_N,
	output HW_RAM_UDM,
	output HW_RAM_LDM,
	output HW_RAM_CK,
	output HW_RAM_CKN,
	output HW_RAM_CKE,
	output HW_RAM_ODT,
	output [12:0] HW_RAM_AD,
	inout  [15:0] HW_RAM_DQ,
	inout  HW_RZQ_PIN,
	inout  HW_ZIO_PIN,
	
	// RS232 Lines
	input  RS232_Uart_RX,
	output RS232_Uart_TX 
);
	
	// REMOVED: data read from/to audio, not switches/LEDs
	/*
	input			[7:0]	switches;
	output		[7:0]	leds;
	*/
	
	// Wires and Register Declarations for modules
	//
	// PicoBlaze Data Lines
	wire			[7:0]	pb_port_id;
	wire			[7:0]	pb_out_port;
	reg			[7:0]	pb_in_port;
	wire			pb_read_strobe;
	wire			pb_write_strobe;
	// PicoBlaze CPU Control Wires
	wire			pb_reset;
	wire			pb_interrupt;
	wire			pb_int_ack;
	
	// Data wires and registers
	reg			[25:0] address;
	reg			[25:0] max_address;
	wire			[15:0] read_out;
	wire			[15:0] audio_data;
	
	// RAM Interface wires
	wire			RAM_status;		// 1 if RAM is ready for R/W
	wire			RAM_reset;
	wire			sysCLK;
	reg			[25:0] max_RAM_address;
	
	// Audio Interface wires
	wire			aud_reset;
	
	// UART wires
	wire			write_to_uart;
	wire			uart_buffer_full;
	wire			uart_data_present;
	reg			read_from_uart;
	wire			uart_reset;
	// UART Data Lines
	// TX does not need a wire, as it is fed directly by pb_out_port
	wire  [7:0]	uart_rx_data;
	
	// REMOVED: LED wires (no need for LEDs)

	// REMOVED: LED Driver and control logic (no need for LEDs)
	
	// RAM Interface
	//
	// RAM interface uses ACTIVE-HIGH reset
	assign RAM_reset = ~RST;
	// RAM Interface instantiation
	mem_interface memory_interface(
		.hw_ram_rasn(HW_RAM_RASN),
		.hw_ram_casn(HW_RAM_CASN),
		.hw_ram_wen(HW_RAM_WEN),
		.hw_ram_ba(HW_RAM_BA),
		.hw_ram_udqs_p(HW_RAM_UDQS_P),
		.hw_ram_udqs_n(HW_RAM_UDQS_N),
		.hw_ram_ldqs_p(HW_RAM_LDQS_P),
		.hw_ram_ldqs_n(HW_RAM_LDQS_N),
		.hw_ram_udm(HW_RAM_UDM),
		.hw_ram_ldm(HW_RAM_LDM),
		.hw_ram_ck(HW_RAM_CK),
		.hw_ram_ckn(HW_RAM_CKN),
		.hw_ram_cke(HW_RAM_CKE),
		.hw_ram_odt(HW_RAM_ODT),
		.hw_ram_ad(HW_RAM_AD),
		.hw_ram_dq(HW_RAM_DQ),
		.hw_rzq_pin(),
		.hw_zio_pin(),
		.CLK(CLK),
		.reset(RAM_reset),
		.read_out(/*<FILL_IN>*/),
		.addr_in(/*<FILL_IN>*/),
		.data_in(/*<FILL_IN>*/),
		.systemCLK(sysCLK),
		.status(RAM_status)
	);
	
	// Clock generator
	//
	// Wires for output clocks from generator
	wire audio_clk;
	wire pb_clk;
	//
	// Takes 37.5MHz clock from ram interface and outputs two 100MHz clocks
	clk_gen clock_generator(
		.CLK_IN1(sysCLK),
		.CLK_OUT1(audio_clk),
		.CLK_OUT2(pb_clk)
	);
	
	// Audio Interface
	//
	// Audio interface uses ACTIVE-HIGH reset
	assign aud_reset = ~RST;
	// Audio Interface instantiation
	sockit_top audio_interface(
		.OSC_100MHz(audio_clk),	// 100 mhz clock from clock wiz
		.AUD_ADCLRCK(AUD_ADCLRCK),
		.AUD_ADCDAT(AUD_ADCDAT),
		.AUD_DACLRCK(AUD_DACLRCK),
		.AUD_DACDAT(AUD_DACDAT),
		.AUD_XCK(AUD_XCK),
		.AUD_BCLK(AUD_BCLK),
		.AUD_I2C_SCLK(AUD_I2C_SCLK),
		.AUD_I2C_SDAT(AUD_I2C_SDAT),
		.AUD_MUTE(AUD_MUTE),
		.PLL_LOCKED(PLL_LOCKED),
		.reset(aud_reset)
	);
	
	// UART and control logic
	//
	// UART expects ACTIVE-HIGH reset	
	assign uart_reset =  ~RST;
	// UART instantiation
	//
	// Within the UART Module (rs232_uart.v), make sure you fill in the
	// appropriate sections.
	rs232_uart UART (
		.tx_data_in(pb_out_port), // The UART only accepts data from PB, so we just tie the PB output to the UART input.
		.write_tx_data(write_to_uart), // Goes high when PB sends write strobe and PORT_ID is the UART write port number
		.tx_buffer_full(uart_buffer_full),
		.rx_data_out(uart_data_rx),
		.read_rx_data_ack(read_from_uart),
		.rx_data_present(uart_data_present),
		.rs232_tx(RS232_Uart_TX),
		.rs232_rx(RS232_Uart_RX),
		.reset(uart_reset),
		.clk(pb_clk)	// 100MHz clock from clock wiz	
	);
	
	// PicoBlaze and control logic
	//
	// PB expects ACTIVE-HIGH reset
	assign pb_reset = ~RST;
	// Disable interrupt by assigning 0 to interrupt
	assign pb_interrupt = 1'b0;
	// PB CPU instantiation
	//
	// Within the PicoBlaze Module (picoblaze.v), make sure you fill in the
	// appropriate sections.
	picoblaze CPU (
		.port_id(pb_port_id),
		.read_strobe(pb_read_strobe),
		.in_port(pb_in_port),
		.write_strobe(pb_write_strobe),
		.out_port(pb_out_port),
		.interrupt(pb_interrupt),
		.interrupt_ack(pb_int_ack),
		.reset(pb_reset),
		.clk(pb_clk)	// Clock signal generated from RAM interface
	);	
	// PB I/O selection/routing
	//
	// Handle PicoBlaze Output Port Logic
	// Output Ports:
	// * leds_out : port 01
	// * uart_data_tx : port 03
	//
	// These signals are effectively "write enable" lines for the UART and LED
	// Driver modules. They must be asserted when PB is outputting to the
	// corresponding port
	assign write_to_leds = pb_write_strobe & (pb_port_id == 8'h01);
	assign write_to_uart = pb_write_strobe & (pb_port_id == 8'h03);
	//
	// Handle PicoBlaze Input Port Logic
	// Input Ports:
	// * switches_in : port 00
	// * uart_data_rx : port 02
	// * uart_data_present : port 04 (1-bit, assigned to LSB)
	// * uart_buffer_full: port 05 (1-bit, assigned to LSB)
	//
	// This process block gets the value of the requested input port device
	// and passes it to PBs in_port. When PB is not requestng data from
	// a valid input port, set the input to static 0.
	always @(posedge pb_clk or posedge pb_reset)
	begin
		if(pb_reset) begin
			pb_in_port <= 0;
			read_from_uart <= 0;
		end else begin
			// Set pb input port to appropriate value
			case(pb_port_id)
				// TODO: INTERFACE PICOBLAZE PORTS
				default: pb_in_port <= 8'h00;
			endcase
			// Set up acknowledge/enable signals.
			//
			// Some modules, such as the UART, need confirmation that the data
			// has been read, since it needs to push it off the queue and make
			// the next byte available. This logic will set the 'read_from'
			// signal high for corresponding ports, as needed. Most input
			// ports will not need this.
			read_from_uart <= pb_read_strobe & (pb_port_id == 8'h04);
		end
	end

	// Debouncers for keypad rows and cols (comment this block out if debouncers are not needed)
	//
	// Wires for debounced inputs
	wire kypd_row0, kypd_row1, kypd_row2, kypd_row3;
	wire kypd_col0, kypd_col1, kypd_col2, kypd_col3;
	//
	// Clock Divider for debouncer input
	wire clk_div;
	clock_divider divider (.clk(pb_clk), .rst(pb_reset), .clk_div(clk_div));
	//
	// Debouncer instantiations for key pads
	debouncer row0 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_ROW[0]), .out(kypd_row0));
	debouncer row1 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_ROW[1]), .out(kypd_row1));
	debouncer row2 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_ROW[2]), .out(kypd_row2));
	debouncer row3 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_ROW[3]), .out(kypd_row3));
	debouncer col0 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_COL[0]), .out(kypd_col0));
	debouncer col1 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_COL[1]), .out(kypd_col1));
	debouncer col2 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_COL[2]), .out(kypd_col2));
	debouncer col3 (.clk(pb_clk), .clock_div(clk_div), .in(KYPD_COL[3]), .out(kypd_col3));
	
	// FSM code block for menu controls and module interfacing
	//
	// Wires and Registers for state functions
	wire  audio_ready;
	wire  data_ready;
	reg   write_en;
	reg   write_req;
	wire  write_ack;
	reg   read_req;
	reg   read_ack;
	reg   [2:0] volume;
	reg   mem_full;
	//
	// Registers for tracks (track[26] is valid bit)
	reg  [26:0] track1;
	reg  [26:0] track2;
	reg  [26:0] track3;
	reg  [26:0] track4;
	reg  [26:0] track5;
	reg  [2:0] track_sel;
	//
	// Keypad controls
	wire kypd1, kypd2, kypd3, kypd4, kypd5, kypd6, kypd0, kypdA, kypdB;
	assign kypd1 = (kypd_row0 & kypd_col0);
	assign kypd2 = (kypd_row0 & kypd_col1);
	assign kypd3 = (kypd_row0 & kypd_col2);
	assign kypdA = (kypd_row0 & kypd_col3);
	assign kypd4 = (kypd_row1 & kypd_col0);
	assign kypd5 = (kypd_row1 & kypd_col1);
	assign kypd6 = (kypd_row1 & kypd_col2);
	assign kypdB = (kypd_row1 & kypd_col3);
	assign kypd0 = (kypd_row3 & kypd_col0);
	//
	// State encoding
	parameter init_state			=	4'b0000;
	parameter menu					=	4'b0001;
	parameter play_menu			=	4'b0010;
	parameter play_state			=	4'b0011;
	parameter record_state		=	4'b0100;
	parameter delete_menu		=	4'b0101;
	parameter delete_state		=	4'b0110;
	parameter delete_all			=	4'b0111;
	parameter mem_full_state	=	4'b1000;
	parameter change_volume		=	4'b1001;
	parameter volume_up			=	4'b1010;
	parameter volume_down		=	4'b1100;
	//
	// State register
	reg [3:0] state = init_state;
	//
	// FSM logic
	always@ (posedge pb_clk) begin
		// Reset case
		if (pb_reset) begin
			write_en 	<= 0;
			write_req 	<= 0;
			read_req 	<= 0;
			read_ack 	<= 0;
			volume 		<= 0;
			address 		<= 0;
			state 		<= init_state;
		end
		// State logic
		// TODO: INSERT PICOBLAZE PORT COMMANDS
		else begin
			case (state)
				// Initialization state
				init_state: begin
					write_en 	<= 0;
					write_req 	<= 0;
					read_req 	<= 0;
					read_ack 	<= 0;
					volume 		<= 0;
					address 		<= 0;
					mem_full 	<= 0;
					track1 		<= 0;
					track2 		<= 0;
					track3 		<= 0;
					track4 		<= 0;
					track5		<= 0;
					state			<= menu;
				end
				// Main menu
				menu: begin
					if (kypd1)					// Play message
						state <= play_menu;
					else if (kypd2)			// Record message
						state <= record_state;
					else if (kypd3)			// Delete message
						state <= delete_menu;
					else if (kypd4)			// Delete all messages
						state <= delete_all;
					else if (kypd5)			// Volume control
						state <= change_volume;
					else							// Loop on menu until selection is made
						state <= menu;
				end
				// Play menu - select a track to play
				play_menu: begin
					if (kypd1) begin
						track_sel 	<= 3'b001;
						state 		<= play_state;
					end
					else if (kypd2) begin
						track_sel	<= 3'b010;
						state			<= play_state;
					end
					else if (kypd3) begin
						track_sel	<= 3'b011;
						state			<= play_state;
					end
					else if (kypd4) begin
						track_sel	<= 3'b100;
						state			<= play_state;
					end
					else if (kypd5) begin
						track_sel	<= 3'b101;
						state			<= play_state;
					end
					else if (kypd0)			// Exit back to the main menu
						state 		<= menu;
					else							// Loop on menu until selection is made
						state			<= play_menu;
				end
				// Play state - begin playing audio from selected track
				play_state: begin
					// TODO: AUDIO PLAYBACK LOGIC
				end
				// Record state
				record_state: begin
					// TODO: RECORD AUDIO LOGIC
					if (mem_full)
						state <= mem_full_state;
					else
						state <= menu;
				end
				// Delete menu - select a track to delete
				delete_menu: begin
					if (kypd1) begin
						track_sel	<= 3'b001;
						state			<= delete_state;
					end
					else if (kypd2) begin
						track_sel	<= 3'b010;
						state			<= delete_state;
					end
					else if (kypd3) begin
						track_sel	<= 3'b011;
						state			<= delete_state;		
					end
					else if (kypd4) begin
						track_sel	<= 3'b100;
						state			<= delete_state;
					end
					else if (kypd5) begin
						track_sel	<= 3'b101;
						state			<= delete_state;
					end
					else if (kypd0)			// Exit back to the main menu
						state			<= menu;
					else							// Loop on menu until selection is made
						state			<= delete_menu;
				end
				// Delete state - sets selected track's valid bit to 0
				delete_state: begin
					if (track_sel == 3'b001)
						track1[26] <= 1'b0;
					else if (track_sel == 3'b010)
						track2[26] <= 1'b0;
					else if (track_sel == 3'b011)
						track3[26] <= 1'b0;
					else if (track_sel == 3'b100)
						track4[26] <= 1'b0;
					else if (track_sel == 3'b101)
						track5[26] <= 1'b0;
					state <= menu;				// Return to main menu after delete is performed
				end
				// Delete all state - sets every track's valid bit to 0
				delete_all: begin
					track1[26] <= 1'b0;
					track2[26] <= 1'b0;
					track3[26] <= 1'b0;
					track4[26] <= 1'b0;
					track5[26] <= 1'b0;
					state 	  <= menu;
				end
				// Memory full state - displays picoblaze message that memory is full and returns to menu
				mem_full_state: begin
					// TODO: INSERT PICOBLAZE PORT
					state <= menu;
				end
				// Change volume state - select 'up' or 'down' to change volume
				change_volume: begin
					if (kypdA)					// Keypad A used as 'volume up' button
						state <= volume_up;
					else if (kypdB)			// Keypad B used as 'volume down' button
						state <= volume_down;
					else if (kypd0)			// Exit back to main menu
						state <= menu;
					else							// Loop on menu until selection is made
						state <= change_volume;
				end
				// Volume up state - increases volume by 1 if it is not already at max
				volume_up: begin
					if (volume < 3'b111)
						volume <= volume + 1;
					// TODO: PICOBLAZE PORT FOR VOLUME INDICATOR
					state <= change_volume;
				end
				// Volume down state - decrease volume by 1 if it is not already at min
				volume_down: begin
					if (volume > 3'b000)
						volume <= volume + 1;
					// TODO: PICOBLAZE PORT FOR VOLUME INDICATOR
					state <= change_volume;
				end
			endcase
		end
	end
	
	assign leds = state;
	
endmodule
