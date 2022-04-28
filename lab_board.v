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
	// Control - clk, rst, etc
	input			reset,			// Remember: ACTIVE LOW!!!
	input			reset_pll,
	input			clk,			// 100 MHz
//	input			ETH_REFCLK,	//	100 MHz
//	input 		pb_clk,		// 100 MHz
	// GPIO
	input	[7:0]	switches,
	output	[7:0]	leds,
	// RS232 Lines
	rs232_rx,
	rs232_tx,
	input [3:0] button,
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
	PLL_LOCKED,
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
	ledRAM,
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
	//output ledRAM,
	//output freqcountout

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
	output PLL_LOCKED;
//	 output PLL_LOCKED2,
	 
 //   input  [3:0] KEY,
//    input  [3:0] SW,
    output [2:0] LED,

	picoblaze_controller picoblaze(
		.switches(switches), 
		.leds(LED), 
		.rs232_tx(rs232_tx), 
		.rs232_rx(rs232_rx), 
		.reset(reset), 
		.clk(CLK)
	);

	codec_interface codec(
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
		.KEY(KEY),
		.SW(SW),
		.LED(LED)
	);

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
	    .CLK(CLK),
    );

endmodule
