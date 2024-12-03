`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/03/2024 01:42:33 PM
// Design Name: 
// Module Name: lab10_tb
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


module lab10_tb;
    reg clk,rst;
    reg [2:0] addr1, addr2, ram_addr;
    wire [7:0] p;
    
    lab10 uut(clk,rst,addr1, addr2, ram_addr, p);
    
    initial begin
    clk = 0; rst = 1; addr1 = 2; addr2 = 1; ram_addr = 0;
        forever #5 clk = ~clk;
    end
    
    initial begin 
    #10 rst = 0;
    #100 rst = 1; ram_addr = 1; addr2 = 6;
    #10 rst = 0;
        #100 $stop;
    end
endmodule
