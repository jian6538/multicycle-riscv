module ControlFSM (
    input  wire        wb_clk,
    input  wire        wb_rst,
    input  wire [6:0]  opcode,
    // Wishbone handshake
    input  wire        mem_ack,
    input  wire [31:0] alu_result,
    // Control outputs
    output reg         pc_write,
    output reg         ir_write,
    output reg         wb_cyc_o,
    output reg         wb_stb_o,
    output reg [31:0]  wb_addr_o,    
    output reg         wb_we_o,     // 0 = read
    output reg         alu_src_a,   // 1 = PC
    output reg [1:0]   alu_src_b,   // 2 = constant 4
    output reg [1:0]   alu_op,      // 00 = ADD
    output reg         RegWrite,
    output reg         MemRead,
    output reg         MemWrite,
    output reg         MemToReg,
    output reg         Branch,
    output reg [1:0]   ALUOp,
    output reg   register_write_final, // Fixed missing comma
    output reg [31:0] alu_out_reg, // Register to hold ALU result for branch decision
    output wire [2:0] fsm_state

);
    

     // Define constants for the opcodes for clarity
    // =============================
    // 1️⃣ State Encoding
    // =============================
    localparam FETCH     = 3'd0;
    localparam DECODE    = 3'd1;
    localparam EXECUTE   = 3'd2;
    localparam MEM       = 3'd3;
    localparam WRITEBACK = 3'd4;
    localparam BRANCH    = 3'd5; // Optional state for branch handling
    
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
    
    reg [2:0] state, next_state;

    // =============================
    // 2️⃣ State Register
    // =============================
    always @(posedge wb_clk) begin
        if (wb_rst) begin
            state <= FETCH;
            alu_out_reg <= 0; // Clear ALU result register on reset
        end else begin
            state <= next_state;
            alu_out_reg <= alu_result; // Capture ALU result for branch decision
        end


    end

    // =============================
    // 3️⃣ Next-State Logic
    // =============================
    always @(*) begin
        next_state = state;

        case(state)
            FETCH: begin
                if (mem_ack)
                    next_state = DECODE; 
            end
            DECODE: begin
                next_state = EXECUTE;
            end
            EXECUTE: begin
                // Check opcode directly rather than using output signals
                if (opcode == OPCODE_LW || opcode == OPCODE_SW) begin
                    next_state = MEM;
                end else if (opcode == OPCODE_RTYPE || opcode == OPCODE_ADDI || 
                             opcode == OPCODE_LUI || opcode == OPCODE_AUIPC || 
                             opcode == OPCODE_JAL || opcode == OPCODE_JALR) begin
                    next_state = WRITEBACK;
                end else if(opcode == OPCODE_BEQ) begin
                    if(alu_result == 0) begin
                        next_state = BRANCH; // Branch taken
                    end else begin
                        next_state = FETCH;  // Branch not taken, go back to fetch
                    end
                end else begin
                    next_state = FETCH; // For unsupported opcodes, just go back to fetch
                end
            end
            MEM: begin
                if (mem_ack) begin
                    if (opcode == OPCODE_LW)       // Load goes to Writeback
                        next_state = WRITEBACK;
                    else                           // Store goes to Fetch
                        next_state = FETCH;
                end
            end
            WRITEBACK: begin
                next_state = FETCH;
            end
            BRANCH: begin
              next_state = FETCH;
            end
            default: next_state = FETCH;
        endcase
    end

    // =============================
    // 4️⃣ Output Logic (Moore/Mealy)
    // =============================
    always @(*) begin
        // ---- Default values ----
        pc_write  = 0;
        ir_write  = 0;
        wb_cyc_o  = 0;
        wb_stb_o  = 0;
        wb_we_o   = 0;  // read
        alu_src_a = 0;
        alu_src_b = 2'b00;
        alu_op    = 2'b00;
        wb_addr_o = 32'b0;
        
        RegWrite  = 0;
        ALUSrc    = 0;
        MemRead   = 0;
        MemWrite  = 0;
        MemToReg  = 0;
        Branch    = 0;
        ALUOp     = 2'b00;
        register_write_final = 0;

        case (state)
            FETCH: begin
                wb_cyc_o = 1;
                wb_stb_o = 1;
                wb_we_o  = 0;        
                alu_src_a = 1; 
                alu_src_b = 2'b10; 
                alu_op = 2'b00;

                if (mem_ack) begin
                    ir_write = 1;    
                    pc_write = 1;    
                end
            end
            
            DECODE: begin
                // Decode state logic
            end

            EXECUTE: begin
                case (opcode)
                    OPCODE_RTYPE: begin
                        alu_src_a = 0;       // Operand A from register (rs1)
                        alu_src_b = 2'b00;   // Operand B from register (rs2)
                        ALUOp     = 2'b10;   // Let ALU Decoder look at funct3/funct7
                    end
                    
                    OPCODE_ADDI: begin
                        alu_src_a = 0;       // Operand A from register (rs1)
                        alu_src_b = 2'b01;   // Operand B from immediate
                        ALUOp     = 2'b00;   // ADD (or 2'b10 if your ALU decoder handles I-type funct3)
                    end
                    
                    OPCODE_LW, OPCODE_SW: begin
                        alu_src_a = 0;       // Operand A from base register (rs1)
                        alu_src_b = 2'b01;   // Operand B from immediate (offset)
                        ALUOp     = 2'b00;   // ALU performs ADD for address calculation
                    end
                    
                    OPCODE_BEQ: begin
                        alu_src_a = 0;       // Operand A from register (rs1)
                        alu_src_b = 2'b00;   // Operand B from register (rs2)
                        ALUOp     = 2'b01;   // ALU performs SUB for equality comparison
                        Branch    = 1;       // Assert branch signal
                    end
                    
                    OPCODE_JAL: begin
                        alu_src_a = 1;       // Operand A from PC
                        alu_src_b = 2'b01;   // Operand B from immediate (offset)
                        ALUOp     = 2'b00;   // ALU performs ADD to calculate jump target
                    end
                    
                    OPCODE_JALR: begin
                        alu_src_a = 0;       // Operand A from register (rs1)
                        alu_src_b = 2'b01;   // Operand B from immediate (offset)
                        ALUOp     = 2'b00;   // ALU performs ADD to calculate jump target
                    end
                    
                    OPCODE_LUI: begin
                        alu_src_a = 0;       // Don't care / unused
                        alu_src_b = 2'b01;   // Operand B from immediate
                        ALUOp     = 2'b11;   // Special ALU op to just pass the immediate through
                    end
                    
                    OPCODE_AUIPC: begin
                        alu_src_a = 1;       // Operand A from PC
                        alu_src_b = 2'b01;   // Operand B from immediate
                        ALUOp     = 2'b00;   // ALU performs ADD (PC + imm)
                    end
                    
                    default: begin
                        // Safe default for undefined opcodes
                        alu_src_a = 0;
                        alu_src_b = 2'b00;
                        ALUOp     = 2'b00;
                        Branch    = 0;
                    end
                endcase
            end
            
            MEM: begin
                if (opcode == OPCODE_LW) MemRead = 1;
                if (opcode == OPCODE_SW) MemWrite = 1;
                
                wb_cyc_o  = 1;
                wb_stb_o  = 1;
                wb_we_o   = MemWrite; 
                wb_addr_o = alu_out_reg; // Use registered ALU result for stable address during memory access
            end
            
            WRITEBACK: begin
                // Only assert register writes here
                if (opcode == OPCODE_LW) MemToReg = 1;
                RegWrite = 1; 
                register_write_final = 1; 
            end
            BRANCH: begin
                alu_src_a = 1;       // Operand A from register (rs1)
                alu_src_b = 2'b01;   // Operand B from register (rs2)
                pc_write  = 1;       // Update PC to branch target
                ALUOp = 2'b00; // No specific ALU operation needed for branch handling
            end
        endcase
    end
    assign fsm_state = state;
endmodule