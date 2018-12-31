module top(input logic			 clk, reset,
			  output logic [31:0] WriteData, Adr,
			  output logic			 MemWrite);
	
	logic [31:0] ReadData;
	
	arm arm(clk, reset, MemWrite, Adr, WriteData, ReadData);
	
	
	mem memoo(clk, MemWrite, Adr, WriteData, ReadData);
	
endmodule

module mem(input  logic clk, we,
				input  logic [31:0] a, wd,
				output logic [31:0] rd);
	logic [31:0] RAM[63:0];
	
	initial
			$readmemh("memfile.dat",RAM);
	
	assign rd = RAM[a[31:2]];
	
	always_ff @(posedge clk)
		if (we) RAM[a[31:2]] <= wd;
endmodule



module arm(input logic         clk, reset,
			  output logic        MemWrite,
			  output logic [31:0] ALUResult, WriteData,
			  input logic  [31:0] ReadData);
	logic [31:0] Instr;		  
	logic [3:0] ALUFlags;
	logic       RegWrite, PCWrite, IRWrite;
	logic [1:0] RegSrc, ImmSrc, ALUControl, ALUSrcB, ResultSrc;
	
	controller c(clk, reset, Instr[31:0], ALUFlags,
					 PCWrite, MemWrite, RegWrite, IRWrite,
					 AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc, ImmSrc, ALUControl);
					 
	datapath dp(clk, reset, Adr, WriteData, ReadData, Instr, ALUFlags, PCWrite, RegWrite, IRWrite,
					AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc, ImmSrc, ALUControl);
					
endmodule

module mainFSM(input logic clk,
					input logic reset,
					input logic [1:0] Op,
					input logic [5:0] Funct,
					output logic IRWrite,
					output logic AdrSrc, ALUSrcA,
					output logic [1:0] ALUSrcB, ResultSrc,
					output logic NextPC, RegW, MemW, Branch, ALUOp);
					
	typedef enum logic [3:0] {S0, S1, S2, S3, S4, S5, S6, S7, S8, S9} statetype;
	statetype state, nextstate; 
	logic[11:0] controls;
	always_ff @(posedge clk, posedge reset)
		if (reset) state <= S0;
		else 		  state <= nextstate;
		
		
	always_comb
		case (state)
			S0:												 nextstate=S1;
					
			S1: if (~Op[1] & Op[0]) 					 nextstate = S2;
				 else if (~Op[1] & ~Op[0] & ~Funct[5]) nextstate = S6;
				 else if (~Op[1] & ~Op[0] & Funct[5])  nextstate = S7;
				 else											 nextstate = S9;
			
			S2: if (Funct[0])								 nextstate = S3;
				 else											 nextstate = S5;
				 
			S3:												 nextstate = S4;
			
			S6: 												 nextstate = S8;
			
			S7: 												 nextstate = S8;
			
			S4:												 nextstate = S0;
			
			S5:												 nextstate = S0;
			
			S8:												 nextstate = S0;
			
			S9:												 nextstate = S0;
			
			default:											 nextstate = S0;
		
		endcase
	
	always_comb
		case (state)
			S0:	controls = 12'b010110001100;
					
			S1: 	controls = 12'b010110000000;
			
			S2: 	controls = 12'b000001000000;
				 
			S3:	controls = 12'b100000000000;											 
			
			S6: 	controls = 12'b000000000001;									 
			
			S7: 	controls = 12'b000001000001;											 
			
			S4:	controls = 12'b001000100000;											 
			
			S5:	controls = 12'b100000010000;											 
			
			S8:	controls = 12'b000000100000;											 
			
			S9:	controls = 12'b010001000010;											 
														 
		
		endcase
	
	assign {AdrSrc, ResultSrc, ALUSrcA, ALUSrcB, RegW, MemW, IRWrite, NextPC, Branch, ALUOp}=controls;
	
endmodule


module controller(input logic clk,
						input logic reset,
						input logic [31:12] Instr,
						input logic [3:0] ALUFlags,
						output logic PCWrite,
						output logic MemWrite,
						output logic RegWrite,
						output logic IRWrite,
						output logic AdrSrc,
						output logic [1:0] RegSrc,
						output logic ALUSrcA,
						output logic [1:0] ALUSrcB,
						output logic [1:0] ResultSrc,
						output logic [1:0] ImmSrc,
						output logic [1:0] ALUControl);
	logic [1:0] FlagW;
	logic       PCS, NextPC, RegW, MemW;
	decoder dec(clk, reset, Instr[27:26], Instr[25:20], Instr[15:12],
					Flagw, PCS, NextPC, RegW, MemW, 
					IRWrite, AdrSrc, RegSrc, ALUSrcA, ALUSrcB, ResultSrc,ImmSrc, ALUControl);
					
	condlogic cl(clk, reset, Instr, ALUFlags, FlagW, PCS, NextPC, RegW, MemW, Instr[31:28], 
					PCWrite, MemWrite, RegWrite);
endmodule

module decoder(input logic clk, reset,
					input logic [1:0] Op,
					input logic [5:0] Funct,
					input logic [3:0] Rd,
					output logic [1:0] FlagW,
					output logic PCS,
					output logic NextPC,	
					output logic RegW, 
					output logic MemW,
					output logic IRWrite,
					output logic AdrSrc,
					output logic [1:0] RegSrc, 
					output logic ALUSrcA,
					output logic [1:0] ALUSrcB,
					output logic [1:0] ResultSrc, 
					output logic [1:0] ImmSrc, 
					output logic [1:0] ALUControl);
	logic [3:0]  InstrDecode;
	logic        Branch, ALUOp;
	
	mainFSM fsm(clk, reset, Op, Funct, IRWrite, AdrSrc, ALUSrcA, ALUSrcB, ResultSrc,
					NextPC, RegW, MemW, Branch, ALUOp);
					
	//Instr Decoder
	
	assign ImmSrc = Op;
	assign RegSrc[1] = Op[0];
	assign RegSrc[0] = Op[1];
		
	//ALU Decoder
	always_comb
	if(ALUOp) begin // which DP Instr
			case(Funct[4:1])
					4'b0100: ALUControl = 2'b00; // ADD
					4'b0010: ALUControl = 2'b01; // SUB
					4'b0000: ALUControl = 2'b10; // AND
					4'b1100: ALUControl = 2'b11; // ORR
					default: ALUControl = 2'bx; // unimplemented
			endcase
			
			// update flags if S bit is set (C&V only for arith)
			FlagW[1] = Funct[0];
			FlagW[0] = Funct[0]&(ALUControl==2'b00|ALUControl==2'b01);
	end else begin
			ALUControl = 2'b00; // add for non-DP instructions
			FlagW = 2'b00; // don't update Flags
	end
	//End of ALU Decoder
	
	// PC Logic
	assign PCS = ((Rd==4'b1111)&RegW)|Branch;
	// End of PC Logic
	
endmodule


module condlogic(input logic 	clk,
					  input logic reset,
					  input logic [31:0] Instr,
					  input logic [3:0] ALUFlags,
					  input logic [1:0] FlagW,
					  input logic PCS,
					  input logic NextPC,
					  input logic RegW,
					  input logic MemW,
					  input logic [3:0] Cond,
					  output logic PCWrite,
					  output logic MemWrite,
					  output logic RegWrite);
	logic [1:0] FlagWrite;
	logic [3:0] Flags;
	logic CondEx, CondEx1;
	
	flopenr #(2)flagreg1(clk, reset, FlagWrite[1], ALUFlags[3:2], Flags[3:2]);
	flopenr #(2)flagreg0(clk, reset, FlagWrite[0], ALUFlags[1:0], Flags[1:0]);
	
	condcheck cc(Cond, Flags, CondEx);	
	assign FlagWrite = FlagW & {2{CondEx}};
	
	flopr #(1)flagreg2(clk, reset, CondEx, CondEx1);
	
	assign RegWrite = RegW & CondEx1;
	assign MemWrite = MemW & CondEx1;
	assign PCWrite = (PCS & CondEx1)||NextPC;
endmodule

module condcheck(input logic [3:0] Cond,
						input logic [3:0] Flags,
						output logic CondEx);
						
	logic neg, zero, carry, overflow, ge;
	
	assign {neg, zero, carry, overflow} = Flags;
	assign ge = (neg == overflow);
	
	always_comb
		case(Cond)
			4'b0000: CondEx = zero; // EQ
			4'b0001: CondEx = ~zero; // NE
			4'b0010: CondEx = carry; // CS
			4'b0011: CondEx = ~carry; // CC
			4'b0100: CondEx = neg; // MI
			4'b0101: CondEx = ~neg; // PL
			4'b0110: CondEx = overflow; // VS
			4'b0111: CondEx = ~overflow; // VC
			4'b1000: CondEx = carry&~zero; // HI
			4'b1001: CondEx = ~(carry&~zero); // LS
			4'b1010: CondEx = ge; // GE
			4'b1011: CondEx = ~ge; // LT
			4'b1100: CondEx = ~zero&ge; // GT
			4'b1101: CondEx = ~(~zero&ge); // LE
			4'b1110: CondEx = 1'b1; // Always
			default: CondEx = 1'bx; // undefined
		endcase
endmodule

module datapath(input logic       clk, reset,
					 output logic [31:0] Adr, WriteData,
					 input logic [31:0] ReadData,
					 output logic [31:0] Instr,
					 output logic [3:0] ALUFlags,
					 input logic PCWrite, RegWrite,
					 input logic IRwrite,
					 input logic AdrSrc,
					 input logic [1:0] RegSrc,
					 input logic ALUSrcA,
					 input logic [1:0] ALUSrcB, ResultSrc,
					 input logic [1:0] ImmSrc, ALUControl);
					 
					 
					 
	logic [31:0] PCNext, PC;
	logic [31:0] ExtImm, A, WriteData1, SrcA1, SrcA, SrcB, Result, ALUOut, Data;
	logic [3:0] RA1, RA2;
	
	
	
	//getting PC
	flopenr #(32) pcreg(clk, reset, PCWrite, Result, PC);
	
	//getting Adr
	mux2 #(32) adrmux(PC, Result, AdrSrc, Adr);
	
	//get data
	flopr #(32) getdata(clk, reset, ReadData, Data);
	
	// register file logic
	flopenr #(32) rdreg(clk, reset, IRWrite, ReadData, Instr);
	mux2 #(4) ra1mux(Instr[19:16], 4'b1111, RegSrc[0], RA1);
	mux2 #(4) ra2mux(Instr[3:0], Instr[15:12], RegSrc[1], RA2);
	regfile	 rf(clk, RegWrite, RA1, RA2, 
					 Instr[15:12], Result, Result,
					 SrcA1, WriteData1);
	
	// Extend
	extend ext(Instr[23:0], ImmSrc, ExtImm);
	
	//getting SrcA				 
	flopr #(32) geta(clk, reset, SrcA1, A);
	mux2 #(32) srcamux(A, PC, ALUSrcA, SrcA);
	//getting WriteData and SrcB
	flopr #(32) wrdata(clk, reset, WriteData1, WriteData);
	mux3 #(32) srcbmux(WriteData, ExtImm, 32'b100, ALUSrcB, SrcB);
	//ALU logic, get ALU result
	alu alu(SrcA, SrcB, ALUControl, ALUResult, ALUFlags);
	
		
	//AluOut
	flopr #(32) aluout(clk, reset, ALUResult, ALUOut);
	//get Result
	mux3 #(32) getres(ALUOut, Data, ALUResult, ResultSrc, Result);
					 
					 
endmodule


//regfile module
module regfile(input logic         clk,
					input logic         we3,
					input logic  [3:0]  ra1, ra2, wa3,
					input logic  [31:0] wd3, r15,
					output logic [31:0] rd1, rd2);
					
	logic [31:0] rf[14:0];
	
	always_ff@(posedge clk)
		if (we3) rf[wa3] <= wd3;
		
	assign rd1 = (ra1 == 4'b1111) ? r15 : rf[ra1];
	assign rd2 = (ra2 == 4'b1111) ? r15 : rf[ra2];
endmodule

//extend module
module extend(input logic [23:0] Instr,
				  input logic [1:0]  ImmSrc,
				  output logic [31:0] ExtImm);
				  
	always_comb
		case(ImmSrc)
		
			2'b00: ExtImm={24'b0, Instr[7:0]};
			
			2'b01: ExtImm={20'b0, Instr[11:0]};
			
			2'b10: ExtImm={{6{Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm=32'bx;
		endcase
endmodule

//resettable flop
module flopr #(parameter WIDTH = 8)
				  (input  logic clk, reset,
				   input  logic [WIDTH-1:0] d,
					output logic [WIDTH-1:0] q);
	always_ff @(posedge clk, posedge reset)
		if (reset) q<=0;
		else		  q<=d;
		
endmodule


//enable flop
module flopenr #(parameter WIDTH = 8)
				    (input  logic clk, reset, en,
				     input  logic [WIDTH-1:0] d,
					  output logic [WIDTH-1:0] q);
	always_ff @(posedge clk, posedge reset)
		if (reset) 		q<=0;
		else if (en)	q<=d;

endmodule

//mux2
module mux2 #(parameter width = 8)
				 (input  logic [width-1:0] d0, d1, 
				  input  logic             s,
				  output logic [width-1:0] y);

   assign y = s ? d1 : d0; 
endmodule


//mux3
module mux3 #(parameter width = 8)
				 (input  logic [width-1:0] d0, d1, d2, 
				  input  logic [1:0] s,
				  output logic [width-1:0] y);
				 
	
	always_comb
		case(s)
			2'b00: y=d0;
			2'b01: y=d1;
			2'b10: y=d2;
			default: y=d0;
		endcase
endmodule

//alu
module alu(input  logic [31:0] a, b,
           input  logic [1:0]  ALUControl,
           output logic [31:0] Result,
           output logic [3:0]  ALUFlags);

  logic        neg, zero, carry, overflow;
  logic [31:0] condinvb;
  logic [32:0] sum;

  assign condinvb = ALUControl[0] ? ~b : b;
  assign sum = a + condinvb + ALUControl[0];

  always_comb
    casex (ALUControl[1:0])
      2'b0?: Result = sum;
      2'b10: Result = a & b;
      2'b11: Result = a | b;
    endcase

  assign neg      = Result[31];
  assign zero     = (Result == 32'b0);
  assign carry    = (ALUControl[1] == 1'b0) & sum[32];
  assign overflow = (ALUControl[1] == 1'b0) & 
                    ~(a[31] ^ b[31] ^ ALUControl[0]) & 
                    (a[31] ^ sum[31]); 
  assign ALUFlags    = {neg, zero, carry, overflow};
endmodule

module adder #(parameter WIDTH=8)
              (input  logic [WIDTH-1:0] a, b,
               output logic [WIDTH-1:0] y);
             
  assign y = a + b;
endmodule
