// ALU Control Unit
// This module decodes the main control unit's ALUOp signal and the
// instruction's funct3/funct7 fields to generate the specific 4-bit
// control signal for the ALU.

module ALUControl (
    // Input Signals
    input  wire [1:0]  ALUOp,       // From the main Control Unit (R-type, I-type, etc.)
    input  wire [2:0]  funct3,      // From the instruction's funct3 field
    input  wire [6:0]  funct7,      // From the instruction's funct7 field

    // Output Signal
    output reg [3:0]  ALUControl   // The 4-bit control code for the ALU
);

    // --- Localparam for ALU Operations ---
    // These should match the parameters in your alu.v file
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


    // --- Combinational Logic for ALU Control Signal ---
    always @(*) begin
        case (ALUOp)
            // For LW, SW, and ADDI, the operation is always ADD.
            2'b00: begin
                ALUControl = OP_ADD;
            end

            // For BEQ, the operation is always SUB (for comparison).
            2'b01: begin
                ALUControl = OP_SUB;
            end

            // For R-type instructions, we must decode funct3 and funct7.
            2'b10: begin
                case (funct3)
                    3'b000: begin // ADD or SUB
                        if (funct7 == 7'b0100000) begin
                            ALUControl = OP_SUB; // SUB instruction
                        end else begin
                            ALUControl = OP_ADD; // ADD instruction
                        end
                    end
                    3'b001: ALUControl = OP_SLL;  // SLL instruction rd = rs1 << rs2 

                    3'b010: ALUControl = OP_SLT;  // SLT instruction rd = (rs1 < rs2) ? 1 : 0
                    3'b011: ALUControl = OP_SLTU; // SLTU instruction rd = (rs1 < rs2) ? 1 : 0 (unsigned comparison)
                    3'b100: ALUControl = OP_XOR;  // XOR instruction  rd = rs1 ^ rs2
                    3'b101: begin // SRL or SRA
                        if (funct7 == 7'b0100000) begin
                            ALUControl = OP_SRA; // SRA instruction rd = rs1 >>> rs2[4:0] (arithmetic right shift) maintains the sign bit for signed numbers
                        end else begin
                            ALUControl = OP_SRL; // SRL instruction rd = rs1 >> rs2[4:0] (logical right shift) fills with zeros for unsigned numbers
                        end
                    end
                    3'b110: ALUControl = OP_OR;   // OR instruction rd = rs1 | rs2
                    3'b111: ALUControl = OP_AND;  // AND instruction rd = rs1 & rs2
                    default: ALUControl = 4'bxxxx; // Should not happen 
                endcase
            end
            
            // Default case handles other ALUOp values, like for LUI.
            // You can adjust this if you add more complex instructions.
            default: begin
                 ALUControl = OP_ADD; // Default to ADD for safety
            end
        endcase
    end

endmodule
