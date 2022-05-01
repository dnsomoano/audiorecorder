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
	// TODO: ADD BUTTON CONTROLS
	
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
	
	// Wires and Register Declarations
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
		.status(/*<FILL_IN>*/)
	);
		
	
	// Audio Interface
	//
	// Audio interface uses ACTIVE-HIGH reset
	assign aud_reset = ~RST;
	// Audio Interface instantiation
	sockit_top audio_interface(
		.OSC_100MHz(OSC_100MHz),
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
		.clk(/*<FILL_IN>*/)	// Clock signal generated from RAM interface
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
	always @(posedge clk or posedge pb_reset)
	begin
		if(pb_reset) begin
			pb_in_port <= 0;
			read_from_uart <= 0;
		end else begin
			// Set pb input port to appropriate value
			case(pb_port_id)
				8'h00: pb_in_port <= /*<FILL_IN>*/;	// audio data to be written to memory, work in progress
				8'h02: pb_in_port <= uart_data_rx;
				8'h04: pb_in_port <= {7'b0000000,uart_data_present};
				8'h05: pb_in_port <= {7'b0000000,uart_buffer_full};
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


endmodule
