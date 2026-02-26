// A Program Counter module for a multi-cycle RISC-V processor
// that updates only when explicitly enabled by the Control FSM.

module ProgramCounter (
  input  wire        clk,
  input  wire        rst,       // Reset signal to initialize PC to 0
  input  wire [31:0] pc_in,     // The next value for the PC 
  input  wire        pc_write,  // Enable signal from Control FSM or Branch Logic
  output wire [31:0] pc_out     // The current value of the PC
);

  // Internal register to hold the PC value.
  reg [31:0] pc_reg;

  // On the rising edge of the clock, update the PC.
  always @(posedge clk) begin
    if (rst) begin
      pc_reg <= 32'h00000000;
    end
    else if (pc_write) begin
      pc_reg <= pc_in;
    end
  end

  // Assign the internal register value to the output.
  assign pc_out = pc_reg;

endmodule