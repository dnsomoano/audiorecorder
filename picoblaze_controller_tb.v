`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   08:39:28 05/04/2022
// Design Name:   picoblaze_controller
// Module Name:   C:/Users/Halberd/audiorecorder/picoblaze_controller_tb.v
// Project Name:  final_lab
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: picoblaze_controller
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module picoblaze_controller_tb;

	// Inputs
	reg OSC_100MHz;
	reg RST;
	reg [3:0] KYPD_COL;
	reg [3:0] KYPD_ROW;
	reg AUD_ADCDAT;
	reg RS232_Uart_RX;

	// Outputs
	wire AUD_DACDAT;
	wire AUD_XCK;
	wire AUD_I2C_SCLK;
	wire AUD_MUTE;
	wire PLL_LOCKED;
	wire HW_RAM_RASN;
	wire HW_RAM_CASN;
	wire HW_RAM_WEN;
	wire [2:0] HW_RAM_BA;
	wire HW_RAM_UDM;
	wire HW_RAM_LDM;
	wire HW_RAM_CK;
	wire HW_RAM_CKN;
	wire HW_RAM_CKE;
	wire HW_RAM_ODT;
	wire [12:0] HW_RAM_AD;
	wire RS232_Uart_TX;

	// Bidirs
	wire AUD_ADCLRCK;
	wire AUD_DACLRCK;
	wire AUD_BCLK;
	wire AUD_I2C_SDAT;
	wire HW_RAM_UDQS_P;
	wire HW_RAM_UDQS_N;
	wire HW_RAM_LDQS_P;
	wire HW_RAM_LDQS_N;
	wire [15:0] HW_RAM_DQ;
	wire HW_RZQ_PIN;
	wire HW_ZIO_PIN;

	// Instantiate the Unit Under Test (UUT)
	picoblaze_controller uut (
		.OSC_100MHz(OSC_100MHz), 
		.RST(RST), 
		.KYPD_COL(KYPD_COL), 
		.KYPD_ROW(KYPD_ROW), 
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
		.HW_RAM_RASN(HW_RAM_RASN), 
		.HW_RAM_CASN(HW_RAM_CASN), 
		.HW_RAM_WEN(HW_RAM_WEN), 
		.HW_RAM_BA(HW_RAM_BA), 
		.HW_RAM_UDQS_P(HW_RAM_UDQS_P), 
		.HW_RAM_UDQS_N(HW_RAM_UDQS_N), 
		.HW_RAM_LDQS_P(HW_RAM_LDQS_P), 
		.HW_RAM_LDQS_N(HW_RAM_LDQS_N), 
		.HW_RAM_UDM(HW_RAM_UDM), 
		.HW_RAM_LDM(HW_RAM_LDM), 
		.HW_RAM_CK(HW_RAM_CK), 
		.HW_RAM_CKN(HW_RAM_CKN), 
		.HW_RAM_CKE(HW_RAM_CKE), 
		.HW_RAM_ODT(HW_RAM_ODT), 
		.HW_RAM_AD(HW_RAM_AD), 
		.HW_RAM_DQ(HW_RAM_DQ), 
		.HW_RZQ_PIN(HW_RZQ_PIN), 
		.HW_ZIO_PIN(HW_ZIO_PIN), 
		.RS232_Uart_RX(RS232_Uart_RX), 
		.RS232_Uart_TX(RS232_Uart_TX)
	);

	// // Clock Generator
	// always begin
	// 	clk = ~clk;
	// 	#5;
	// end

	// Clock Generator
	always begin
		OSC_100MHz = ~OSC_100MHz;
		#5;
	end

	initial begin
		// Initialize Inputs
		OSC_100MHz = 0;
		RST = 0;
		KYPD_COL = 0;
		KYPD_ROW = 0;
		AUD_ADCDAT = 0;
		RS232_Uart_RX = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
		// * Simulate pressing keypad button 1.
		// * Play Message
		KYPD_ROW[0] = 1;
		KYPD_COL[0] = 1;
		
		#100;

		KYPD_ROW[0] = 0;
		KYPD_COL[0] = 0;

		#100;

		// * Simulate pressing keypad button 2.
		// * Record Message
		KYPD_ROW[0] = 1;
		KYPD_COL[1] = 1;

		#100;


	end
      
endmodule

