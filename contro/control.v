// Main Control Unit for a Single-Cycle RISC-V Processor

module ControlUnit (
    // Input: The opcode from the instruction
    input  wire [6:0] opcode,

    // Outputs: All the control signals for the datapath
    output reg        RegWrite,
    output reg        ALUSrc,
    output reg        MemRead,
    output reg        MemWrite,
    output reg        MemToReg,
    output reg        Branch,
    output reg [1:0]  ALUOp
);

    // Define constants for the opcodes for clarity
    localparam OPCODE_RTYPE = 7'b0110011;
    localparam OPCODE_LW    = 7'b0000011;
    localparam OPCODE_SW    = 7'b0100011;
    localparam OPCODE_BEQ   = 7'b1100011;
    localparam OPCODE_ADDI  = 7'b0010011;
    localparam OPCODE_JAL   = 7'b1101111;
    localparam OPCODE_JALR  = 7'b1100111;
    localparam OPCODE_LUI   = 7'b0110111;
    localparam OPCODE_AUIPC = 7'b0010111;


    // Combinational logic to generate control signals based on opcode
    always @(*) begin
        case (opcode)
            OPCODE_RTYPE: begin
                RegWrite = 1;
                ALUSrc   = 0; // Operand B from register file 1（immediate）0(r2)
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0; // Result comes from ALU
                Branch   = 0;
                ALUOp    = 2'b10; // ALU Decoder will look at funct fields
            end
            OPCODE_ADDI: begin
                RegWrite = 1;
                ALUSrc   = 1;  // Use immediate
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;  // Result from ALU
                Branch   = 0;
                ALUOp    = 2'b00; // ALU will do ADD
            end
            OPCODE_JAL: begin
                RegWrite = 1;      // Write PC+4 to rd
                ALUSrc   = 0;      // Don't care
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;      // ALU can compute PC+imm
                Branch   = 0;      // PC change handled externally
                ALUOp    = 2'b00;  // ADD for PC + imm
            end
            OPCODE_JALR: begin
                RegWrite = 1;
                ALUSrc   = 1;      // Use immediate for offset
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;
                Branch   = 0;
                ALUOp    = 2'b00;
            end
            OPCODE_LUI: begin
                RegWrite = 1;
                ALUSrc   = 1;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;      // Result is immediate, may require bypass
                Branch   = 0;
                ALUOp    = 2'b11;  // Special ALU operation (may be passthrough)
            end
            OPCODE_AUIPC: begin
                RegWrite = 1;
                ALUSrc   = 1;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;
                Branch   = 0;
                ALUOp    = 2'b00;  // ALU adds PC + imm
            end


            OPCODE_LW: begin
                RegWrite = 1;
                ALUSrc   = 1; // Operand B from immediate value
                MemRead  = 1;
                MemWrite = 0;
                MemToReg = 1; // Result comes from Data Memory
                Branch   = 0;
                ALUOp    = 2'b00; // ALU should perform an ADD for address calculation
            end
            OPCODE_SW: begin
                RegWrite = 0; // No register write for a store
                ALUSrc   = 1;
                MemRead  = 0;
                MemWrite = 1;
                MemToReg = 0; // Don't care
                Branch   = 0;
                ALUOp    = 2'b00; // ALU should perform an ADD for address calculation
            end
            OPCODE_BEQ: begin
                RegWrite = 0;
                ALUSrc   = 0;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0; // Don't care
                Branch   = 1;
                ALUOp    = 2'b01; // ALU should perform a SUB for comparison
            end
            default: begin // For unknown opcodes, set all signals to a safe "off" state
                RegWrite = 0;
                ALUSrc   = 0;
                MemRead  = 0;
                MemWrite = 0;
                MemToReg = 0;
                Branch   = 0;
                ALUOp    = 2'b00;
            end
        endcase
    end

endmodule
