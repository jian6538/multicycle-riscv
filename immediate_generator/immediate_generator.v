// Immediate Generator for RISC-V
// This module decodes the instruction to generate the correct sign-extended
// immediate value based on the instruction type (I, S, B, U, J).

module ImmediateGenerator (
    input  wire [31:0] instruction,
    output reg  [31:0] immediate
);

    // Extract fields from the instruction based on RISC-V formats
    wire [6:0] opcode = instruction[6:0];
    wire [11:0] i_imm = instruction[31:20];
    wire [11:5] s_imm_11_5 = instruction[31:25];
    wire [4:0] s_imm_4_0 = instruction[11:7];
    wire b_imm_12 = instruction[31];
    wire [10:5] b_imm_10_5 = instruction[30:25];
    wire [4:1] b_imm_4_1 = instruction[11:8];
    wire b_imm_11 = instruction[7];
    wire [31:12] u_imm = instruction[31:12];
    wire j_imm_20 = instruction[31];
    wire [19:12] j_imm_19_12 = instruction[19:12];
    wire j_imm_11_j = instruction[20];
    wire [10:1] j_imm_10_1 = instruction[30:21];

    // Define opcodes for clarity
    localparam OPCODE_RTYPE = 7'b0110011;
    localparam OPCODE_ITYPE = 7'b0010011;
    localparam OPCODE_LOAD  = 7'b0000011;
    localparam OPCODE_STYPE = 7'b0100011;
    localparam OPCODE_BTYPE = 7'b1100011;
    localparam OPCODE_JALR  = 7'b1100111;
    localparam OPCODE_JAL   = 7'b1101111;
    localparam OPCODE_LUI   = 7'b0110111;
    localparam OPCODE_AUIPC = 7'b0010111;

    // Combinational logic to generate the correct immediate
    always @(*) begin
        case (opcode)
            // I-Type (ADDI, etc.) and Loads (LW)
            OPCODE_ITYPE, OPCODE_LOAD, OPCODE_JALR: begin
                // Sign-extend the 12-bit immediate
                immediate = {{20{i_imm[11]}}, i_imm}; 
            end

            // S-Type (SW)
            OPCODE_STYPE: begin
                // Reassemble and sign-extend the 12-bit immediate
                immediate = {{20{s_imm_11_5[11]}}, s_imm_11_5, s_imm_4_0};
            end

            // B-Type (BEQ)
            OPCODE_BTYPE: begin
                // Reassemble, sign-extend, and shift the 13-bit immediate
                immediate = {{19{b_imm_12}}, b_imm_12, b_imm_11, b_imm_10_5, b_imm_4_1, 1'b0};
            end

            // U-Type (LUI, AUIPC)
            OPCODE_LUI, OPCODE_AUIPC: begin
                // Reassemble the 20-bit immediate, lower 12 bits are 0
                immediate = {u_imm, 12'b0};
            end

            // J-Type (JAL)
            OPCODE_JAL: begin
                // Reassemble, sign-extend, and shift the 21-bit immediate
                immediate = {{11{j_imm_20}},j_imm_20, j_imm_19_12, j_imm_11_j, j_imm_10_1, 1'b0};
            end

            // Default for R-Type or others that don't use an immediate
            default: begin
                immediate = 32'hDEADBEEF; // Default to a debug value
            end
        endcase
    end

endmodule
