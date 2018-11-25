//16 BIT SINGLE CYCLE MIPS PROCESSOR in BEHAVIOURAL MODELLING

//16CO145  SUMUKHA PK
//16CO234  PRAJVAL M

/*
Abstract: 
	The following code works as a 16 BIT MIPS PROCESSOR complete with CPU, Control Unit, ALU , Register File, DATA Memory and other
	components used in simple circultry like decoders, MUX, Shift Registers, D Flip Flop etc
	To give the instructions go ROM File and enter the instructions in the ROM.
*/

/*
Functionalities: 
	This MIPS processor has 13 working functions. They are:
		ADD, SUB, AND, OR, SLT, JR 	- R-type
		BEQ, ADDI, SLTI, LW, SW 	- I-type
		J, JAL           			- J-type
*/

/*
Description of code:
	The below code is done mainly in Data Flow modelling 
	module with its functionality are as follows	
		VerilogBM_145_234 -> Gives the functionality and control to the CPU
		data_memory 	-> Provides memory for data
		ALUControl		-> Controls the ALU operations
		JRControl_unit	-> Controls the Jump instructions
		Control 		-> Control Unit of CPU
		adder			-> Full Adder
		D_FF			-> Shift Register
		alu				-> ALU of the processor
		mux2X5to5		-> five 2 to 1 MUX
		reg_file		-> Register File
		decoder			-> 5 to 32 decoder 
		Pc_reg			-> PC-register
	Its made sure that while variables are named small letters are used
*/

module VerilogBM_145_234( input clk,reset,  
                           output[15:0] pc_out, alu_result,reg3,reg4,reg1,reg2,reg7
   );  
 reg[15:0] pc_current;  
 wire signed[15:0] pc_next,pc2;  
 wire [15:0] instr;  
 wire[1:0] reg_dst,mem_to_reg,alu_op;  
 wire jump,branch,mem_read,mem_write,alu_src,reg_write     ;  
 wire     [2:0]     reg_write_dest;  
 wire     [15:0] reg_write_data;  
 wire     [2:0]     reg_read_addr_1;  
 wire     [15:0] reg_read_data_1;  
 wire     [2:0]     reg_read_addr_2;  
 wire     [15:0] reg_read_data_2;  
 wire [15:0] sign_ext_im,read_data2,zero_ext_im,imm_ext;  
 wire JRControl;  
 wire [2:0] ALU_Control;  
 wire [15:0] ALU_out;  
 wire zero_flag;  
 wire signed[15:0] im_shift_1, PC_j, PC_beq, PC_4beq,PC_4beqj,PC_jr;  
 wire beq_control;  
 wire [14:0] jump_shift_1;  
 wire [15:0]mem_read_data;  
 wire [15:0] no_sign_ext;  
 wire sign_or_zero;  
 // PC   
 always @(posedge clk or posedge reset)  
 begin
      if(reset)   
           pc_current <= 16'd0;  
      else  
           pc_current <= pc_next;  
 end  
 // PC + 2   
 assign pc2 = pc_current + 16'd2;  
 // instruction memory  
 instr_mem instrucion_memory(.pc(pc_current),.instruction(instr));  
 // jump shift left 1  
 assign jump_shift_1 = {instr[13:0],1'b0};  
 // control unit  
 control control_unit(.reset(reset),.opcode(instr[15:13]),.reg_dst(reg_dst)  
                ,.mem_to_reg(mem_to_reg),.alu_op(alu_op),.jump(jump),.branch(branch),.mem_read(mem_read),  
                .mem_write(mem_write),.alu_src(alu_src),.reg_write(reg_write),.sign_or_zero(sign_or_zero));  
 // multiplexer regdest  
 assign reg_write_dest = (reg_dst==2'b10) ? 3'b111: ((reg_dst==2'b01) ? instr[6:4] :instr[9:7]);  
 // register file  
 assign reg_read_addr_1 = instr[12:10];  
 assign reg_read_addr_2 = instr[9:7];  
 register_file reg_file(.clk(clk),.rst(reset),.reg_write_en(reg_write),  
 .reg_write_dest(reg_write_dest),  
 .reg_write_data(reg_write_data),  
 .reg_read_addr_1(reg_read_addr_1),  
 .reg_read_data_1(reg_read_data_1),  
 .reg_read_addr_2(reg_read_addr_2),  
 .reg_read_data_2(reg_read_data_2), 
 .reg3(reg3),  
 .reg4(reg4),
 .reg1(reg1),
 .reg2(reg2),
 .reg7(reg7));  
 // sign extend  
 assign sign_ext_im = {{9{instr[6]}},instr[6:0]};  
 assign zero_ext_im = {{9{1'b0}},instr[6:0]};  
 assign imm_ext = (sign_or_zero==1'b1) ? sign_ext_im : zero_ext_im;  
 // JR control  
 JR_Control JRControl_unit(.alu_op(alu_op),.funct(instr[3:0]),.JRControl(JRControl));       
 // ALU control unit  
 ALUControl ALU_Control_unit(.ALUOp(alu_op),.Function(instr[3:0]),.ALU_Control(ALU_Control));  
 // multiplexer alu_src  
 assign read_data2 = (alu_src==1'b1) ? imm_ext : reg_read_data_2;  
 // ALU   
 alu alu_unit(.a(reg_read_data_1),.b(read_data2),.alu_control(ALU_Control),.result(ALU_out),.zero(zero_flag));  
 // immediate shift 1  
 assign im_shift_1 = {imm_ext[14:0],1'b0};  
 //  
 assign no_sign_ext = ~(im_shift_1) + 1'b1;  
 // PC beq add  
 assign PC_beq = (im_shift_1[15] == 1'b1) ? (pc2 - no_sign_ext): (pc2 +im_shift_1);  
 // beq control  
 assign beq_control = branch & zero_flag;  
 // PC_beq  
 assign PC_4beq = (beq_control==1'b1) ? PC_beq : pc2;  
 // PC_j  
 assign PC_j = {pc2[15],jump_shift_1};  
 // PC_4beqj  
 assign PC_4beqj = (jump == 1'b1) ? PC_j : PC_4beq;  
 // PC_jr  
 assign PC_jr = reg_read_data_1;  
 // PC_next  
 assign pc_next = (JRControl==1'b1) ? PC_jr : PC_4beqj;  
 // data memory  
 data_memory datamem(.clk(clk),.mem_access_addr(ALU_out),  
 .mem_write_data(reg_read_data_2),.mem_write_en(mem_write),.mem_read(mem_read),  
 .mem_read_data(mem_read_data));  
 // write back  
 assign reg_write_data = (mem_to_reg == 2'b10) ? pc2:((mem_to_reg == 2'b01)? mem_read_data: ALU_out);  
 // output  
 assign pc_out = pc_current;  
 assign alu_result = ALU_out;  
 endmodule
 
 
 


//DATA MEMORY MODULE
 module data_memory  
 (  
		input	clk,  
      // address input, shared by read and write port  
		input [15:0] mem_access_addr,  
      // write port  
		input [15:0] mem_write_data,  
		input mem_write_en,  
		input mem_read,  
      // read port  
      output  [15:0] mem_read_data  
 );  
      
	  
		integer i;  
		reg [15:0] ram [255:0];  
		wire [7 : 0] ram_addr = mem_access_addr[8 : 1];  
		initial begin  
			for(i=0;i<256;i=i+1)  
				ram[i] <= 16'd0;  
		end  
		always @(posedge clk) begin  
			if (mem_write_en)  
				ram[ram_addr] <= mem_write_data;  
		end  
		assign mem_read_data = (mem_read==1'b1) ? ram[ram_addr]: 16'd0;   
 endmodule
 
 
 //ALUControl
 module ALUControl( ALU_Control, ALUOp, Function);  
	output reg[2:0] ALU_Control;  
	input [1:0] ALUOp;  
	input [3:0] Function;  
	wire [5:0] ALUControlIn;  
	assign ALUControlIn = {ALUOp,Function};  
	always @(ALUControlIn)  
	casex (ALUControlIn)  
	  6'b11xxxx: ALU_Control=3'b000;  
	  6'b10xxxx: ALU_Control=3'b100;  
	  6'b01xxxx: ALU_Control=3'b001;  
	  6'b000000: ALU_Control=3'b000;  
	  6'b000001: ALU_Control=3'b001;  
	  6'b000010: ALU_Control=3'b010;  
	  6'b000011: ALU_Control=3'b011;  
	  6'b000100: ALU_Control=3'b100;  
	  default: ALU_Control=3'b000;  
	  endcase  
endmodule 


// Verilog code for JR control unit
module JR_Control( 
	input[1:0] alu_op, 
    input [3:0] funct,
    output JRControl
);
	assign JRControl = ({alu_op,funct}==6'b001000) ? 1'b1 : 1'b0;
endmodule


//Control Unit
module control( input[2:0] opcode,  
    input reset,  
    output reg[1:0] reg_dst,mem_to_reg,alu_op,  
    output reg jump,branch,mem_read,mem_write,alu_src,reg_write,sign_or_zero                      
);  
 always @(*)  
 begin  
	if(reset == 1'b1) begin  
			reg_dst = 2'b00;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b00;  
			jump = 1'b0;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b0;  
			reg_write = 1'b0;  
			sign_or_zero = 1'b1;  
	end  
	else begin  
	case(opcode)   
	3'b000: begin // add  
			reg_dst = 2'b01;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b00;  
			jump = 1'b0;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b0;  
			reg_write = 1'b1;  
			sign_or_zero = 1'b1;  
			end  
	3'b001: begin // sli  
			reg_dst = 2'b00;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b10;  
			jump = 1'b0;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b1;  
			reg_write = 1'b1;  
			sign_or_zero = 1'b0;  
			end  
	3'b010: begin // j  
			reg_dst = 2'b00;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b00;  
			jump = 1'b1;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b0;  
			reg_write = 1'b0;  
			sign_or_zero = 1'b1;  
			end  
	3'b011: begin // jal  
			reg_dst = 2'b10;  
			mem_to_reg = 2'b10;  
			alu_op = 2'b00;  
			jump = 1'b1;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b0;  
			reg_write = 1'b1;  
			sign_or_zero = 1'b1;  
			end  
	3'b100: begin // lw  
			reg_dst = 2'b00;  
			mem_to_reg = 2'b01;  
			alu_op = 2'b11;  
			jump = 1'b0;  
			branch = 1'b0;  
			mem_read = 1'b1;  
			mem_write = 1'b0;  
			alu_src = 1'b1;  
			reg_write = 1'b1;  
			sign_or_zero = 1'b1;  
			end  
	3'b101: begin // sw  
			reg_dst = 2'b00;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b11;  
			jump = 1'b0;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b1;  
			alu_src = 1'b1;  
			reg_write = 1'b0;  
			sign_or_zero = 1'b1;  
			end  
	3'b110: begin // beq  
			reg_dst = 2'b00;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b01;  
			jump = 1'b0;  
			branch = 1'b1;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b0;  
			reg_write = 1'b0;  
			sign_or_zero = 1'b1;  
			end  
	3'b111: begin // addi  
			reg_dst = 2'b00;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b11;  
			jump = 1'b0;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b1;  
			reg_write = 1'b1;  
			sign_or_zero = 1'b1;  
			end  
	default: begin  
			reg_dst = 2'b01;  
			mem_to_reg = 2'b00;  
			alu_op = 2'b00;  
			jump = 1'b0;  
			branch = 1'b0;  
			mem_read = 1'b0;  
			mem_write = 1'b0;  
			alu_src = 1'b0;  
			reg_write = 1'b1;  
			sign_or_zero = 1'b1;  
			end  
	endcase  
	end  
 end  
 endmodule



 
 //Full Adder
 module adder(sum,cout,a,b,cin);  
	input  a,b,cin;  
	output cout,sum;  
	// sum = a xor b xor cin  
	xor #(50) (sum,a,b,cin);  
	// carry out = a.b + cin.(a+b)  
	and #(50) and1(c1,a,b);  
	or #(50) or1(c2,a,b);  
	and #(50) and2(c3,c2,cin);  
	or #(50) or2(cout,c1,c3);  
 endmodule
 

 
 //Shift Register
 module D_FF (q, d, rst_n, clk,init_value);  
	output q;  
	input d, rst_n, clk,init_value;  
	reg q; // Indicate that q is stateholding  
	always @(posedge clk or negedge rst_n)  
	if (~rst_n)  
		q <= init_value;     // On reset, set to 0  
	else  
		q <= d; // Otherwise out = d   
 endmodule
 


 //ALU
module alu(       
	input          [15:0]     a,          //src1  
	input          [15:0]     b,          //src2  
	input          [2:0]     alu_control,     //function sel  
	output     reg     [15:0]     result,          //result       
	output zero  
	);  
always @(*)  
begin   
	case(alu_control)  
	3'b000: result = a + b; // add  
	3'b001: result = a - b; // sub  
	3'b010: result = a & b; // and  
	3'b011: result = a | b; // or  
	3'b100: begin if (a<b) result = 16'd1;  
					else result = 16'd0;  
					end  
	default:result = a + b; // add  
	endcase  
end  
assign zero = (result==16'd0) ? 1'b1: 1'b0;  
endmodule 

//REG file

module register_file  
(  
	input         clk,  
	input         rst,  
	// write port  
	input         reg_write_en,  
	input         [2:0]  reg_write_dest,  
	input         [15:0] reg_write_data,  
	//read port 1  
	input         [2:0]  reg_read_addr_1,  
	output        [15:0] reg_read_data_1,  
	//read port 2  
	input         [2:0]  reg_read_addr_2,  
	output        [15:0] reg_read_data_2,  
	output 		  [15:0] reg3,reg4,reg1,reg2,reg7
);  
    reg [15:0] reg_array [7:0];
	reg [15:0] reg3,reg4,reg1,reg2,reg7;
		// write port  
		//reg [2:0] i;  
		always @ (posedge clk or posedge rst) begin  
			if(rst) begin  
				reg_array[0] <= 16'b0;  
                reg_array[1] <= 16'b0;  
                reg_array[2] <= 16'b0;  
                reg_array[3] <= 16'b0;  
                reg_array[4] <= 16'b0;  
                reg_array[5] <= 16'b0;  
                reg_array[6] <= 16'b0;  
                reg_array[7] <= 16'b0;       
			end  
			else begin  
                if(reg_write_en) begin  
                     reg_array[reg_write_dest] <= reg_write_data;  
                end  
			end 
				reg1 = reg_array[1];
				reg2 = reg_array[2];
				reg3 = reg_array[3];
				reg4 = reg_array[4];
				reg7 = reg_array[7];
			end  
		assign reg_read_data_1 = ( reg_read_addr_1 == 0)? 16'b0 : reg_array[reg_read_addr_1];  
		assign reg_read_data_2 = ( reg_read_addr_2 == 0)? 16'b0 : reg_array[reg_read_addr_2];  
endmodule 


//PC  REGISTER

module PC_Reg(PCOut,PCin,reset,clk);  
 output [31:0] PCOut;  
 input [31:0] PCin;  
 input reset,clk;  
 D_FF dff0(PCOut[0],PCin[0],reset,clk,1'b0);  
 D_FF dff1(PCOut[1],PCin[1],reset,clk,1'b0);  
 D_FF dff2(PCOut[2],PCin[2],reset,clk,1'b0);  
 D_FF dff3(PCOut[3],PCin[3],reset,clk,1'b0);  
 D_FF dff4(PCOut[4],PCin[4],reset,clk,1'b0);  
 D_FF dff5(PCOut[5],PCin[5],reset,clk,1'b0);  
 D_FF dff6(PCOut[6],PCin[6],reset,clk,1'b0);  
 D_FF dff7(PCOut[7],PCin[7],reset,clk,1'b0);  
 D_FF dff8(PCOut[8],PCin[8],reset,clk,1'b0);  
 D_FF dff9(PCOut[9],PCin[9],reset,clk,1'b0);  
 D_FF dff10(PCOut[10],PCin[10],reset,clk,1'b0);  
 D_FF dff11(PCOut[11],PCin[11],reset,clk,1'b0);  
 D_FF dff12(PCOut[12],PCin[12],reset,clk,1'b0);  
 D_FF dff13(PCOut[13],PCin[13],reset,clk,1'b0);  
 D_FF dff14(PCOut[14],PCin[14],reset,clk,1'b0);  
 D_FF dff15(PCOut[15],PCin[15],reset,clk,1'b0);  
 D_FF dff16(PCOut[16],PCin[16],reset,clk,1'b0);  
 D_FF dff17(PCOut[17],PCin[17],reset,clk,1'b0);  
 D_FF dff18(PCOut[18],PCin[18],reset,clk,1'b0);  
 D_FF dff19(PCOut[19],PCin[19],reset,clk,1'b0);  
 D_FF dff20(PCOut[20],PCin[20],reset,clk,1'b0);  
 D_FF dff21(PCOut[21],PCin[21],reset,clk,1'b0);  
 D_FF dff22(PCOut[22],PCin[22],reset,clk,1'b0);  
 D_FF dff23(PCOut[23],PCin[23],reset,clk,1'b0);  
 D_FF dff24(PCOut[24],PCin[24],reset,clk,1'b0);  
 D_FF dff25(PCOut[25],PCin[25],reset,clk,1'b0);  
 D_FF dff26(PCOut[26],PCin[26],reset,clk,1'b0);  
 D_FF dff27(PCOut[27],PCin[27],reset,clk,1'b0);  
 D_FF dff28(PCOut[28],PCin[28],reset,clk,1'b0);  
 D_FF dff29(PCOut[29],PCin[29],reset,clk,1'b0);  
 D_FF dff30(PCOut[30],PCin[30],reset,clk,1'b0);  
 D_FF dff31(PCOut[31],PCin[31],reset,clk,1'b0);  
 endmodule


//DECODER

module decoder(WriteEn,RegWrite, WriteRegister);  
 input RegWrite;  
 input [4:0] WriteRegister;  
 output [31:0] WriteEn;  
 wire [31:0] OE; // Output Enable  
 dec5to32 dec(OE,WriteRegister);  
 assign WriteEn[0]=0;  
  and #(50) gate1(WriteEn[1],OE[1],RegWrite);  
  and #(50) gate2(WriteEn[2],OE[2],RegWrite);  
  and #(50) gate3(WriteEn[3],OE[3],RegWrite);  
  and #(50) gate4(WriteEn[4],OE[4],RegWrite);  
  and #(50) gate5(WriteEn[5],OE[5],RegWrite);  
  and #(50) gate6(WriteEn[6],OE[6],RegWrite);  
  and #(50) gate7(WriteEn[7],OE[7],RegWrite);  
  and #(50) gate8(WriteEn[8],OE[8],RegWrite);  
  and #(50) gate9(WriteEn[9],OE[9],RegWrite);  
  and #(50) gate10(WriteEn[10],OE[10],RegWrite);  
  and #(50) gate11(WriteEn[11],OE[11],RegWrite);  
  and #(50) gate12(WriteEn[12],OE[12],RegWrite);  
  and #(50) gate13(WriteEn[13],OE[13],RegWrite);  
  and #(50) gate14(WriteEn[14],OE[14],RegWrite);  
  and #(50) gate15(WriteEn[15],OE[15],RegWrite);  
  and #(50) gate16(WriteEn[16],OE[16],RegWrite);  
  and #(50) gate17(WriteEn[17],OE[17],RegWrite);  
  and #(50) gate18(WriteEn[18],OE[18],RegWrite);  
  and #(50) gate19(WriteEn[19],OE[19],RegWrite);  
  and #(50) gate20(WriteEn[20],OE[20],RegWrite);  
  and #(50) gate21(WriteEn[21],OE[21],RegWrite);  
  and #(50) gate22(WriteEn[22],OE[22],RegWrite);  
  and #(50) gate23(WriteEn[23],OE[23],RegWrite);  
  and #(50) gate24(WriteEn[24],OE[24],RegWrite);  
  and #(50) gate25(WriteEn[25],OE[25],RegWrite);  
  and #(50) gate26(WriteEn[26],OE[26],RegWrite);  
  and #(50) gate27(WriteEn[27],OE[27],RegWrite);  
  and #(50) gate28(WriteEn[28],OE[28],RegWrite);  
  and #(50) gate29(WriteEn[29],OE[29],RegWrite);  
  and #(50) gate30(WriteEn[30],OE[30],RegWrite);  
  and #(50) gate31(WriteEn[31],OE[31],RegWrite);  
 endmodule  
 module andmore(g,a,b,c,d,e);  
  output g;  
  input a,b,c,d,e;  
  and #(50) and1(f1,a,b,c,d),  
       and2(g,f1,e);  
 endmodule  
 module dec5to32(Out,Adr);  
 input [4:0] Adr; // Adr=Address of register  
 output [31:0] Out;  
 not #(50) Inv4(Nota, Adr[4]);  
 not #(50) Inv3(Notb, Adr[3]);  
 not #(50) Inv2(Notc, Adr[2]);  
 not #(50) Inv1(Notd, Adr[1]);  
 not #(50) Inv0(Note, Adr[0]);  
 andmore a0(Out[0], Nota,Notb,Notc,Notd,Note); // 00000  
 andmore a1(Out[1], Nota,Notb,Notc,Notd,Adr[0]); // 00001  
 andmore a2(Out[2], Nota,Notb,Notc,Adr[1],Note); //00010  
 andmore a3(Out[3], Nota,Notb,Notc,Adr[1],Adr[0]);  
 andmore a4(Out[4], Nota,Notb,Adr[2],Notd,Note);  
 andmore a5(Out[5], Nota,Notb,Adr[2],Notd,Adr[0]);  
 andmore a6(Out[6], Nota,Notb,Adr[2],Adr[1],Note);  
 andmore a7(Out[7], Nota,Notb,Adr[2],Adr[1],Adr[0]);  
 andmore a8(Out[8],  Nota,Adr[3],Notc,Notd,Note);  
 andmore a9(Out[9],  Nota,Adr[3],Notc,Notd,Adr[0]);  
 andmore a10(Out[10], Nota,Adr[3],Notc,Adr[1],Note);  
 andmore a11(Out[11], Nota,Adr[3],Notc,Adr[1],Adr[0]);  
 andmore a12(Out[12], Nota,Adr[3],Adr[2],Notd,Note);  
 andmore a13(Out[13], Nota,Adr[3],Adr[2],Notd,Adr[0]);  
 andmore a14(Out[14], Nota,Adr[3],Adr[2],Adr[1],Note);  
 andmore a15(Out[15], Nota,Adr[3],Adr[2],Adr[1],Adr[0]);  
 andmore a16(Out[16], Adr[4],Notb,Notc,Notd,Note);  
 andmore a17(Out[17], Adr[4],Notb,Notc,Notd,Adr[0]);  
 andmore a18(Out[18], Adr[4],Notb,Notc,Adr[1],Note);  
 andmore a19(Out[19], Adr[4],Notb,Notc,Adr[1],Adr[0]);  
 andmore a20(Out[20], Adr[4],Notb,Adr[2],Notd,Note);  
 andmore a21(Out[21], Adr[4],Notb,Adr[2],Notd,Adr[0]);  
 andmore a22(Out[22], Adr[4],Notb,Adr[2],Adr[1],Note);  
 andmore a23(Out[23], Adr[4],Notb,Adr[2],Adr[1],Adr[0]);  
 andmore a24(Out[24], Adr[4],Adr[3],Notc,Notd,Note);  
 andmore a25(Out[25], Adr[4],Adr[3],Notc,Notd,Adr[0]);  
 andmore a26(Out[26], Adr[4],Adr[3],Notc,Adr[1],Note);  
 andmore a27(Out[27], Adr[4],Adr[3],Notc,Adr[1],Adr[0]);  
 andmore a28(Out[28], Adr[4],Adr[3],Adr[2],Notd,Note);  
 andmore a29(Out[29], Adr[4],Adr[3],Adr[2],Notd,Adr[0]);  
 andmore a30(Out[30], Adr[4],Adr[3],Adr[2],Adr[1],Note);  
 andmore a31(Out[31], Adr[4],Adr[3],Adr[2],Adr[1],Adr[0]); // 11111  
 endmodule

