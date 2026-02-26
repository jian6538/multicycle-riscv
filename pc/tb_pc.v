// tb_pc.v
module tb_pc;

  reg clk;
  reg rst;
  wire [31:0] pc;


  pc my_pc(clk, rst, pc);

  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk; // 10ns clock period
  end

  // Test sequence
  initial begin
    $dumpfile("pc.vcd");
    $dumpvars(0, tb_pc);

  
    rst = 1;
    #15; 
    rst = 0;
    #100; 


  end

endmodule
