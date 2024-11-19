`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/19/2024 02:32:33 PM
// Design Name: 
// Module Name: rom_tb
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


module rom_tb;
reg [2:0] addr;
wire [3:0] d_out;

rom uut(addr,d_out);
initial
begin
    for(integer i = 0; i<8; i=i+1)
    begin
        addr = i; #10;
    end
    $stop;
end
endmodule
