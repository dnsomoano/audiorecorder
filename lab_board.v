`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:33:43 04/04/2022 
// Design Name: 
// Module Name:    lab_board 
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
module lab_board(
	SW, 
	LED,
	RS232_Uart_TX, 
	RS232_Uart_RX, 
	BTN,
	CLK,	
	// * sockit_top I/O
    AUD_ADCLRCK,
    AUD_ADCDAT,
    AUD_DACLRCK,
    AUD_DACDAT,

    AUD_XCK,
    AUD_BCLK,

    AUD_I2C_SCLK,
    AUD_I2C_SDAT,

    AUD_MUTE,
	//PLL_LOCKED,
    // * Memory Interface I/O
	hw_ram_rasn,
	hw_ram_casn,
	hw_ram_wen,
	hw_ram_ba,
	hw_ram_udqs_p,
	hw_ram_udqs_n,
	hw_ram_ldqs_p,
	hw_ram_ldqs_n,
	hw_ram_udm,
	hw_ram_ldm,
	hw_ram_ck,
	hw_ram_ckn,
	hw_ram_cke,
	hw_ram_odt,
	hw_ram_ad,
	hw_ram_dq,
	hw_rzq_pin,
	hw_zio_pin,
	status,
    );

    // * Memory Interface I/O
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
	output 	status;

	// * sockit_top I/O
    inout  AUD_ADCLRCK;
    input  AUD_ADCDAT;

    inout  AUD_DACLRCK;
    output AUD_DACDAT;

    output AUD_XCK;
    inout  AUD_BCLK;

    output AUD_I2C_SCLK;
    inout  AUD_I2C_SDAT;

    output AUD_MUTE;
	//output PLL_LOCKED;
	 
	// Top-level Inputs and Outputs
	// These connect directly to FPGA pins via the pin map
	//
	// Control - clk, rst, etc
	// input			reset;			// Remember: ACTIVE LOW!!!
	input [3:0]		BTN;
	input			CLK;			// 100 MHz
	// GPIO
	input	[7:0]	SW;
	output	[7:0]	LED;
	// RS232 Lines
	input			RS232_Uart_RX;
	output			RS232_Uart_TX;
	
	// Wires and Register Declarations
	//
	// PicoBlaze Data Lines
	wire	[7:0]	pb_port_id;
	wire	[7:0]	pb_out_port;
	reg		[7:0]	pb_in_port;
	wire			pb_read_strobe;
	wire			pb_write_strobe;
	// PicoBlaze CPU Control Wires
	wire			pb_reset;
	wire			pb_interrupt;
	wire			pb_int_ack;
	
	// UART wires
	wire			write_to_uart;
	wire			uart_buffer_full;
	wire			uart_data_present;
	reg				read_from_uart;
	wire			uart_reset;
	// UART Data Lines
	// TX does not need a wire, as it is fed directly by pb_out_port
	wire	[7:0]	uart_rx_data;
	
	// LED wires
	wire write_to_leds;
	wire led_reset;

	wire clk_picoblaze_clk;
	wire clk_uart_clk;
	wire clk_led_driver;


	// CODEC wires
	wire clk_to_codec_clk;

	wire clk_to_ram_interface;
	

	// LED Driver and control logic
	//
	// LED driver expects ACTIVE-HIGH reset
	assign led_reset = ~BTN[0];
	//assign led_reset = ~reset;

	assign clk_led_driver = CLK;
	// LED driver instantiation
	led_driver_wrapper led_driver (
		.led_value(pb_out_port),
		.leds(leds),
		.write_to_leds(write_to_leds),
		.reset(led_reset),
		.clk(clk_led_driver)
	);
 //   input  [3:0] KEY,
//    input  [3:0] SW,
    //output [2:0] LED,
	 
	 // TODO wire from picoblaze switches to AUD_ADCDAT for recording and AUD_DACDAT for playback

	// picoblaze_controller picoblaze(
	// 	.switches(switches), 
	// 	.leds(LED), 
	// 	.rs232_tx(rs232_tx), 
	// 	.rs232_rx(rs232_rx), 
	// 	.reset(reset), 
	// 	.clk(CLK)
	// );

	// UART and control logic
	//
	// UART expects ACTIVE-HIGH reset	
	assign uart_reset =  ~BTN[0];
	assign clk_uart_clk = CLK;
	//assign uart_reset =  ~reset;
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
		.clk(clk_uart_clk)
	);	
	
	// PicoBlaze and control logic
	//
	// PB expects ACTIVE-HIGH reset
	assign pb_reset = ~BTN[0];
	// Disable interrupt by assigning 0 to interrupt
	assign pb_interrupt = 1'b0;

	assign clk_picoblaze_clk = CLK;
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
		.clk(clk_picoblaze_clk)
	);	

	assign clk_to_codec_clk = CLK;

	codec_interface codec(
		.OSC_100MHz(clk_to_codec_clk),
		.AUD_ADCLRCK(AUD_ADCLRCK),
		.AUD_ADCDAT(AUD_ADCDAT),
		.AUD_DACLRCK(AUD_DACLRCK),
		.AUD_DACDAT(AUD_DACDAT),
		.AUD_XCK(AUD_XCK),
		.AUD_BCLK(AUD_BCLK),
		.AUD_I2C_SCLK(AUD_I2C_SCLK),
		.AUD_I2C_SDAT(AUD_I2C_SDAT),
		.AUD_MUTE(AUD_MUTE),
		.PLL_LOCKED(LED[4]),
		.KEY(SW[3:0]),
		// * May have to declare switches reverse order like in sockit_top.ucf
		.SW(SW[7:4]),
		.LED(LED[3:0])
	);

	assign clk_to_ram_interface = CLK;

    mem_interface RAMinterface(
	    .hw_ram_rasn(hw_ram_rasn),
	    .hw_ram_casn(hw_ram_casn),
	    .hw_ram_wen(hw_ram_wen),
	    .hw_ram_ba(hw_ran_ba),
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
	    .CLK(clk_to_ram_interface),
		.reset(BTN[0]),
		.leds(),
		.switches(SW),
		.dip_switches(dip_switches),
		.status(status)
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
	always @(posedge CLK or posedge pb_reset)
	begin
		if(pb_reset) begin
			pb_in_port <= 0;
			read_from_uart <= 0;
		end else begin
			// Set pb input port to appropriate value
			case(pb_port_id)
				8'h00: pb_in_port <= SW;
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
