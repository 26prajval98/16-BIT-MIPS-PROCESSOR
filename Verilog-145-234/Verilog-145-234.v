`timescale 1ns / 1ps
//16 BIT SINGLE CYCLE MIPS PROCESSOR Testbench

//16CO145  SUMUKHA PK
//16CO234  PRAJVAL M

/*
Abstract: 
	The following code gives the testbench for the Behavioural and dataflow model for the verilog code
	Dataflow instantiation has been commented out. Please remove the comment part to access whichever model you want.
	Don't forget to link the ROM
*/

/*
Functionalities: 
		The testbench does not input instructions to the Processor.
		Instructions to the processor are given by the ROM file included with the files.
*/

/*
Description of code:
	The below code is done mainly in Data Flow modelling 
	module with its functionality are as follows	
		Verilog_145_234 -> Testbench module
		GTKWAVE and $monitor outputs are given
	Its made sure that while variables are named small letters are used
*/

module Verilog_145_234;  
	// Inputs  
	reg clk;  
	reg reset;  
	// Outputs  
	wire [15:0] pc_out;  
	wire [15:0] alu_result,reg1,reg2,reg3,reg4,reg7;  
	// Instantiate the Unit Under Test (UUT)  
	/*VerilogBM_145_234 uut (  
		.clk(clk),   
		.reset(reset),   
		.pc_out(pc_out),   
		.alu_result(alu_result), 
		.reg1(reg1),
		.reg2(reg2),
		.reg3(reg3),  
		.reg4(reg4),
		.reg7(reg7)
	);*/ 
	VerilogDM_145_234 uut (  
		.clk(clk),   
		.reset(reset),   
		.pc_out(pc_out),   
		.alu_result(alu_result), 
		.reg1(reg1),
		.reg2(reg2),
		.reg3(reg3),  
		.reg4(reg4),
		.reg7(reg7)
	);
	initial begin  
	   clk = 0;  
	   forever #10 clk = ~clk;  
	end  
	initial begin  
		//$dumpfile("VerilogBM-145-234.vcd");
		$dumpfile("VerilogDM-145-234.vcd");
		$dumpvars(0,Verilog_145_234);
		// Initialize Inputs  
		$monitor ("register 1=%d, register 2=%d, register 3=%d, register 4=%d register 7 = %d", reg1,reg2,reg3,reg4,reg7);  
		reset = 1;  
		// Wait 100 ns for global reset to finish  
		#100;  
		reset = 0;  
		// Add stimulus here
		#2000 $finish;
	end  
endmodule  