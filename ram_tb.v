`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 01:57:15 PM
// Design Name: 
// Module Name: ram_tb
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


module ram_tb;
    reg clk, w, rst;
    reg[7:0] d_in;
    reg [2:0] addr;
    wire [7:0] d_out;
    
    ram uut(w,clk,rst,d_in,addr, d_out);
    
    initial
    begin
        w = 0; rst = 0; clk = 0; d_in = 0; addr = 0;
        forever #5 clk = ~clk;
    end
    
    initial
    begin
        #10 rst = 1;
        #10 rst = 0; w = 1;
        for(integer i = 0; i<8; i=i+1)
        begin
            #10 addr = i; d_in = i*i-i+10;
        end
        #10 w=0; addr = 3;
        #10 addr = 7;
        #10 addr = 2;
        
        
        #10 rst = 1;
        #10 rst = 0; addr = 3;
        #10 addr = 7;
        #10 addr = 2;
        
        #10 $stop;
        
    end
endmodule
