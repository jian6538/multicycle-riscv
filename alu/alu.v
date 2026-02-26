// ALU (Arithmetic Logic Unit)
// Performs arithmetic and logic operations based on a 4-bit control signal.

module alu(
	input  wire [31:0] operand_a,    // First operand
	input  wire [31:0] operand_b,    // Second operand
	input  wire [3:0]  ALUControl,   // Specific 4-bit operation code from ALU Control
	output reg  [31:0] result,       // 32-bit result of the operation
	output wire        z             // Zero flag (1 if result is 0)
);

    // Define constants for the ALU operations for clarity
    localparam OP_ADD  = 4'b0000;
    localparam OP_SUB  = 4'b0001;
    localparam OP_SLL  = 4'b0010;
    localparam OP_SLT  = 4'b0011;
    localparam OP_SLTU = 4'b0100;
    localparam OP_XOR  = 4'b0101;
    localparam OP_OR   = 4'b0110;
    localparam OP_AND  = 4'b0111;
    localparam OP_SRL  = 4'b1000;
    localparam OP_SRA  = 4'b1001;

    // Combinational logic to perform the selected operation
    always @(*) begin 
        case(ALUControl)
            OP_ADD : result = operand_a + operand_b;
            OP_SUB : result = operand_a - operand_b;
            OP_SLL : result = operand_a << operand_b[4:0]; 
            OP_SLT : result = ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0;
            OP_SLTU: result = (operand_a < operand_b) ? 32'd1 : 32'd0;
            OP_XOR : result = operand_a ^ operand_b;
            OP_OR  : result = operand_a | operand_b;
            OP_AND : result = operand_a & operand_b;
            OP_SRL : result = operand_a >> operand_b[4:0];
            OP_SRA : result = $signed(operand_a) >>> operand_b[4:0];
            default: result = 32'hDEADBEEF; // Should not be reached
        endcase
    end

    // The zero flag 'z' is set to 1 if the result is exactly 0.
    assign z = (result == 32'b0);

endmodule
