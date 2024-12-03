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
    input clk, rst,
    input [2:0] addr1, addr2,ram_addr,
    output [7:0] ram_out
    );
    wire w_rf, DA, SA, SB, w_ram;
    wire [2:0] adr, st_out;
    wire [3:0] rom_d_out, A, B;
    wire [7:0] mult_out;
    cu uut(clk,rst,addr1, addr2, w_rf, adr, DA, SA, SB, st_out, w_ram);
    rom uut1(adr,rom_d_out);
    RF uut2(A, B, SA, SB, rom_d_out, DA, w_rf, rst, clk);
    comb_mult uut3(A, B, mult_out);
    ram uut4(w_ram, clk, rst, mult_out, ram_addr, ram_out);
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




module cu ( 

input clk, reset,
  input [2:0] adr1,
  input [2:0] adr2,
  output reg w_rf,
  output reg [2:0] adr,
  output reg DA,  SA,SB,
  output reg [2:0] st_out,
  //output reg done,
  output reg w_ram
);

   
  
parameter S0_idle = 0 , S1_send_adr1 = 1 , S2_send_adr2 = 2 ,S3_multiply = 3 ,S4_write_ram = 4,S5_read_ram=5 ;
reg [2:0] PS,NS ;

    always@(posedge clk or posedge reset)
        begin
            if(reset)
                PS <= S0_idle;   
            else    
                PS <= NS ;
        end  


		  

    always@(*)
        begin 
            
            case(PS)
				S0_idle:begin
				NS = S1_send_adr1;
                                w_rf <=1'b1;
                                w_ram <=1'b1;
            st_out <= 3'b000;
				end
				
				S1_send_adr1:begin	
				w_rf <=1'b1;
				adr<=adr1;
				DA <=1'b0;
				SA <=1'b0;
				SB <=1'b1;
				st_out <= 3'b001;
				NS = S2_send_adr2;
				end
				
				S2_send_adr2:begin
				w_rf <=1'b1;
				adr<=adr2;
				NS = S3_multiply;
				DA <=1'b1;
				SA <=1'b0;
				SB <=1'b1;
            st_out <= 3'b010;
			   end
			
		                S3_multiply: begin
				NS = 	S4_write_ram;
                                st_out <= 3'b011;
				w_ram<=1;
			        end

                                S4_write_ram: begin
                                st_out <= 3'b100;
				NS = 	S5_read_ram;
				end
				
				S5_read_ram: begin
                                w_ram<=0;
                                //done <=1;
                                st_out <= 3'b101;
				if(!reset) begin
				NS = 	S5_read_ram;
				end
				else begin
				NS = S0_idle;
				end
				end
				endcase
				end
				
			

endmodule


module comb_mult(
    input [3:0] a, b,
    output [7:0] p
    );
    wire [0:0] a0b0, a1b0, a2b0, a3b0, a0b1, a1b1, a2b1, a3b1, a0b2, a1b2, a2b2, a3b2, a0b3, a1b3, a2b3, a3b3;
    wire[7:0] pp1, pp2, pp3, pp4;
    assign a0b0 = a[0]*b[0]; assign a1b0 = a[1]*b[0]; assign a2b0 = a[2]*b[0]; assign a3b0 = a[3]*b[0]; 
    assign a0b1 = a[0]*b[1]; assign a1b1 = a[1]*b[1]; assign a2b1 = a[2]*b[1]; assign a3b1 = a[3]*b[1];  
    assign a0b2 = a[0]*b[2]; assign a1b2 = a[1]*b[2]; assign a2b2 = a[2]*b[2]; assign a3b2 = a[3]*b[2]; 
    assign a0b3 = a[0]*b[3]; assign a1b3 = a[1]*b[3]; assign a2b3 = a[2]*b[3]; assign a3b3 = a[3]*b[3];   
    
    assign pp1 = {4'b0, a3b0, a2b0, a1b0, a0b0};
    assign pp2 = {3'b0, a3b1, a2b1, a1b1, a0b1, 1'b0};
    assign pp3 = {2'b0, a3b2, a2b2, a1b2, a0b2, 2'b0};
    assign pp4 = {1'b0, a3b3, a2b3, a1b3, a0b3, 3'b0};
    
    
    assign p = pp1+pp2+pp3+pp4;
    
endmodule






    
