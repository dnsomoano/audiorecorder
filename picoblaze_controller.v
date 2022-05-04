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
	input  OSC_100MHz,		// CLOCK 100MHz
	input	 RST,					// Reset: ACTIVE LOW!!!!!!
	input [3:0] KYPD_COL,
	input	[3:0] KYPD_ROW,
	
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
	
	// RAM Interface wires
	wire			RAM_reset;
	
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
	
	// * Picoblaze States
	reg play, pause, record, delete, delete_all;
	// * Boolean Flags to hold states.
	reg is_playing, is_recording, storage_full;
	// * Menu options to interact with keypad buttons.
	reg main_menu, play_menu, recording_menu, delete_msg_menu, 
		del_all_menu, vol_menu;
	
	// * Registers used by the FSM.
	reg play_msg_1, play_msg_2, play_msg_3, play_msg_4, play_msg_5;
	reg recording_msg_1, recording_msg_2, recording_msg_3, recording_msg_4, recording_msg_5;
	reg count, del_1, del_2, del_3, del_4, del_5;

	reg [3:0] volume_setting;
	reg vol_up, vol_down;

	wire [15:0] audio_out;

	initial begin
		play <= 0;
		pause <= 0;
		record <= 0;
		is_playing <= 0;
		is_recording <= 0;
		delete <=0;
		delete_all <= 0;
		state <= 8'h00;
		storage_full <= 0;
		volume_setting <= 1;
		vol_up <= 0;
		vol_down <= 0;
		main_menu <= 1;
		play_menu <= 0;
		recording_menu <= 0;
		delete_msg_menu <= 0;
		del_all_menu <= 0;
		vol_menu <= 0;
	end

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
		.CLK(OSC_100MHz),
		.reset(RAM_reset),
		.read_out(/*<FILL_IN>*/),
		.addr_in(/*<FILL_IN>*/),
		.data_in(/*<FILL_IN>*/),
		.status(RAM_status)
	);
		
	
	// Audio Interface
	//
	// Audio interface uses ACTIVE-HIGH reset
	assign aud_reset = ~RST;
	// Audio Interface instantiation
	sockit_top audio_interface(
		.OSC_100MHz(/*<FILL_IN>*/),	// 100 mhz clock from clock wiz
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
		.rs232_tx(rs232_tx),
		.rs232_rx(rs232_rx),
		.reset(uart_reset),
		.clk(/*<FILL_IN>*/)	// 100MHz clock from clock wiz	
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
		.clk(/*<FILL_IN>*/)	// Clock signal generated from RAM interface
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
	assign play_menu = kypd_row0 & kypd_col0;
	assign recording_menu = kypd_row0 & kypd_col1;
	assign delete_msg_menu = kypd_row0 & kypd_col2;
	assign del_all_menu = kypd_row1 & kypd_col0;
	assign vol_menu = kypd_row1 & kypd_col1;
	assign write_to_leds = pb_write_strobe & (pb_port_id == 8'h06);
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
	always @(posedge clk or posedge pb_reset)
	begin
		if(pb_reset) begin
			pb_in_port <= 0;
			read_from_uart <= 0;
		end else begin
			// Set pb input port to appropriate value
			case(pb_port_id)
				// TODO: INTERFACE PICOBLAZE PORTS
				8'h00: pb_in_port <= main_menu;
				8'h01: pb_in_port <= play_menu;
				8'h02: pb_in_port <= uart_data_rx;
				8'h03: pb_in_port <= recording_menu;
				8'h04: pb_in_port <= {7'b0000000,uart_data_present};
				8'h05: pb_in_port <= {7'b0000000,uart_buffer_full};
				8'h06: pb_in_port <= delete_msg_menu;
				8'h07: pb_in_port <= del_all_menu;
				8'h08: pb_in_port <= vol_menu;
				8'h09: pb_in_port <= is_playing;
				8'h0A: pb_in_port <= is_recording;
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
			// Initialize registers
			if (write_to_leds) begin
				main_menu <= (pb_out_port == 8'h00);
				play_menu <= (pb_out_port == 8'h01);
				recording_menu <= (pb_out_port == 8'h03);
				delete_msg_menu <= (pb_out_port == 8'h06);
				del_all_menu <= (pb_out_port == 8'h07);
				vol_menu <= (pb_out_port == 8'h08);
				is_playing <= (pb_out_port == 8'h09);
				is_recording <= (pb_out_port == 8'h0A);
			end
			// Main menu
			if (main_menu) begin
				if (play_menu) begin
					play_msg_1 <= (pb_out_port == 8'h01);
					play_msg_2 <= (pb_out_port == 8'h03);
					play_msg_3 <= (pb_out_port == 8'h06);
					play_msg_4 <= (pb_out_port == 8'h07);
					play_msg_5 <= (pb_out_port == 8'h08);
				end
				else if (recording_menu) begin
					recording_msg_1 <= (pb_out_port == 8'h01);
					recording_msg_2 <= (pb_out_port == 8'h03);
					recording_msg_3 <= (pb_out_port == 8'h06);
					recording_msg_4 <= (pb_out_port == 8'h07);
					recording_msg_5 <= (pb_out_port == 8'h08);
				end
				else if (delete_msg_menu) begin
					del_1 <= (pb_out_port == 8'h01);
					del_2 <= (pb_out_port == 8'h03);
					del_3 <= (pb_out_port == 8'h06);
					del_4 <= (pb_out_port == 8'h07);
					del_5 <= (pb_out_port == 8'h08);
				end
				else if (del_all_menu) begin
					count = 1;
					while (count < 6) begin
						case (count)
							1: pb_in_port <= del_1;
							2: pb_in_port <= del_2;
							3: pb_in_port <= del_3;
							4: pb_in_port <= del_4;
							5: pb_in_port <= del_5;
						endcase
						count = count + 1;
					end
				end
				else if (vol_menu) begin
					vol_up <= (pb_out_port == 8'h01);
					vol_down <= (pb_out_port == 8'h03);
				end
			end
			else if (play_menu) begin
				case (play_menu)
					play_msg_1: 
					play_msg_2: 
					play_msg_3: 
					play_msg_4: 
					play_msg_5: pb_in_port <= is_playing;
					default: pb_in_port <= play_menu;
				endcase
			end
			else if (recording_menu) begin
				case (recording_menu)
					recording_msg_1: 
					recording_msg_2: 
					recording_msg_3: 
					recording_msg_4: 
					recording_msg_5: pb_in_port <= is_recording;
					default: pb_in_port <= recording_menu;
				endcase
			end
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
	clock_divider divider (.clk(OSC_100MHz), .rst(pb_reset), .clk_div(clk_div));
	//
	// Debouncer instantiations for key pads
	debouncer row0 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_ROW[0]), .out(kypd_row0));
	debouncer row1 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_ROW[1]), .out(kypd_row1));
	debouncer row2 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_ROW[2]), .out(kypd_row2));
	debouncer row3 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_ROW[3]), .out(kypd_row3));
	debouncer col0 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_COL[0]), .out(kypd_col0));
	debouncer col1 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_COL[1]), .out(kypd_col1));
	debouncer col2 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_COL[2]), .out(kypd_col2));
	debouncer col3 (.clk(OSC_100MHz), .clk_div(clk_div), .in(KYPD_COL[3]), .out(kypd_col3));
	
	// FSM code block for menu controls and module interfacing
	//
	// Wires and Registers for state functions
	wire	RAM_status;		// 1 if RAM is ready for R/W
	
	
endmodule
