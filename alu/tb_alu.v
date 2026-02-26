// Code your testbench here
// or browse Examples
// tb_alu.v
module tb_alu;

  reg  [31:0] A, B;
  reg  [3:0]  ALUControl;
  wire [31:0]  Result;
  wire        Zero;

  alu my_alu (A, B, ALUControl, Result, Zero);

  initial begin
    $dumpfile("alu.vcd");
    $dumpvars(0, tb_alu);

    // Test ADD: 5 + 10 = 15
    A = 32'd5; B = 32'd10; ALUControl = 4'b0000; #10;
    if (Result !== 15) $display("ADD failed!");

    // Test SUB: 10 - 5 = 5
    A = 32'd10; B = 32'd5; ALUControl = 4'b0001; #10;
    if (Result !== 5) $display("SUB failed!");
    
    // Test SUB leading to zero
    A = 32'd5; B = 32'd5; ALUControl = 4'b0001; #10;
    if (Zero !== 1) $display("Zero flag failed!");

    // Test SLT: 5 < 10 is true (1)
    A = 32'd5; B = 32'd10; ALUControl = 4'b0100; #10;
    if (Result !== 1) $display("SLT (true) failed!");
    
    // Test SLT: 10 < 5 is fQalse (0)
    A = 32'd10; B = 32'd5; ALUControl = 4'b0100; #10;
    if (Result !== 0) $display("SLT (false) failed!");

    $finish;
  end

endmodule
