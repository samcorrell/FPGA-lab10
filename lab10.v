`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:50:59 PM
// Design Name: 
// Module Name: lab10
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module lab10(

    );
endmodule


module ram(
    input w, clk, rst,
    input [7:0] d_in,
    input [2:0] addr,
    output reg [7:0] d_out
    );
    
    reg [7:0]mem[0:7];
    
    integer i;
    always @(posedge clk)
    begin
        if(rst)
        begin
            for(i = 0; i<8; i= i+1)
            begin
                mem[i] <= 8'b0;
            end
        end
        else begin
        if(w)
            mem[addr] <= d_in;
        else
            d_out<=mem[addr];
        end
    end
endmodule


module rom(
    input [2:0] addr,
    output reg[3:0] d_out
    );
    always @(*)
    begin
        case(addr)
            0: d_out = 4'b0;
            1: d_out = 4'b1100;
            2: d_out = 4'b0110;
            3: d_out = 4'b0111;
            4: d_out = 4'b1000;
            5: d_out = 4'b0001;
            6: d_out = 4'b1101;
            7: d_out = 4'b1110;
            default: d_out = 4'bx;
        endcase
    end
endmodule


/*Here I have created a register file using only 2 registers just t make simple. So 1 bit
is enough to represent the register Address
DA- 1bit adress of register where I want to write data
SA- 1bit adress of register, from where I want to read data
SB- 1bit adress of register, from where I want to read data
For most mathemtical operation, we need two data, so 2 registers 
source address SA, SB and 1 destination DA adress to write data*/ 

module RF(A, B, SA, SB, D, DA, W, rst, clk);
	output [3:0]A; // A bus
	output [3:0]B; // B bus
	input SA; // Select A - A Address
	input SB; // Select B - B Address
	input [3:0]D; // Data input
	input DA; // Data destination address
	input W; // write enable
	input rst; // positive logic asynchronous reset
	input clk;
	
	wire [1:0]load_enable;
	wire [3:0]R00, R01;
	
	
	Decoder1to2 decoder (load_enable, DA, W);
	RegisterNbit reg00 (D,R00,load_enable[0], rst, clk); //D-in, R00-out
	RegisterNbit reg01 (D,R01,load_enable[1], rst, clk);
	Mux2to1Nbit muxA (A,R00, R01, SA);
	Mux2to1Nbit muxB (B,R00, R01,SB); 

endmodule

module RegisterNbit(D, Q,  L, R, clock);
	parameter N = 4; // number of bits
	output reg [N-1:0]Q; // registered output
	input [N-1:0]D; // data input
	input L; // load enable
	input R; // positive logic asynchronous reset
	input clock; // positive edge clock
	
	always @(posedge clock or posedge R) begin
		if(R)
			Q <= 0;
		else if(L)
			Q <= D;
		else
			Q <= Q;
	end
endmodule

module Decoder1to2(m, S, en);
	input S; // select
	input en; // enable (positive logic)
	output [1:0]m; // 32 minterms
	
	assign m[0] = ~S&en;
	assign m[1] = S&en;
	
endmodule

module Mux2to1Nbit(o, i1,i2, s);
   input [3:0] i1,i2;
   input  s;
   output reg  [3:0] o;
 
always @(s or i1 or i2)
begin
   case (s)
      1'b0 : o = i1;
      1'b1 : o = i2;
      default : o = 4'b0;
   endcase
end
endmodule

    
