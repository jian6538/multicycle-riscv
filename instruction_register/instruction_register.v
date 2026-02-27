module InstructionRegister (
    input  wire        clk,
    input  wire        rst,
    input ir_write,
    input  wire [31:0] instruction_in,
    output reg  [31:0] instruction_out
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            instruction_out <= 32'b0; // Reset to zero on reset
        end else if (ir_write) begin
            instruction_out <= instruction_in; // Load new instruction on clock edge

        end
    end
endmodule
