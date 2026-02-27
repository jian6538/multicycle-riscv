// RiscV_MultiCycle: Final top-level module for the multi-cycle processor
module RiscV_MultiCycle (
    input  wire        clk,
    input  wire        rst,

    // Wishbone Master Interface (Unified Memory)
    output wire [31:0] wb_adr_o,
    output wire [31:0] wb_dat_o,
    input  wire [31:0] wb_dat_i,
    output wire        wb_we_o,
    output wire        wb_cyc_o,
    output wire        wb_stb_o,
    input  wire        wb_ack_i
);

    // --- Internal Wires and Signals ---
    wire [31:0] pc_out;           // Current PC value
    reg  [31:0] old_pc;           // Latch for branch/jump calculations
    wire [31:0] instruction;      // Instruction from Instruction Register
    
    // Decoded instruction fields
    wire [6:0]  opcode;
    wire [4:0]  rd, rs1, rs2;
    wire [2:0]  funct3;
    wire [6:0]  funct7;

    // Control signals
    wire        reg_write, mem_read, mem_write, mem_to_reg, branch, pc_write, ir_write, register_write_final;
    wire [1:0]  alu_op;
    wire [3:0]  alu_ctrl_signal;
    wire        alu_src_a;        
    wire [1:0]  alu_src_b;        
    wire [2:0]  fsm_state;        // Current state from FSM

    // Data path signals
    wire [31:0] read_data1, read_data2;
    wire [31:0] immediate;        
    wire [31:0] alu_result;
    wire        alu_zero;
    reg  [31:0] write_data;       // Changed to reg for use in always block
    wire [31:0] alu_out_reg;      // Stable ALU result from FSM
    
    // MUX output wires
    wire [31:0] alu_src_final_a;  
    wire [31:0] alu_src_final_b;  

    // ==========================================
    // 1. Internal Registers for Persistence
    // ==========================================
    always @(posedge clk) begin
        if (rst) 
            old_pc <= 32'b0;
        else if (ir_write) 
            old_pc <= pc_out; // Capture address of current instruction
    end


    // ==========================================
    // 2. Module Instantiations
    // ==========================================
    
    ControlFSM control_fsm (
        .wb_clk(clk),
        .wb_rst(rst),
        .opcode(opcode),
        .mem_ack(wb_ack_i),
        .alu_result(alu_result),
        .pc_write(pc_write),
        .ir_write(ir_write),
        .wb_cyc_o(wb_cyc_o),
        .wb_stb_o(wb_stb_o),
        .wb_we_o(wb_we_o),     
        .alu_src_a(alu_src_a),   
        .alu_src_b(alu_src_b),   
        .alu_op(alu_op),
        .MemRead(mem_read),
        .MemWrite(mem_write),
        .MemToReg(mem_to_reg),
        .Branch(branch),
        .RegWrite(reg_write),                  
        .register_write_final(register_write_final), // Fixed comma here!
        .alu_out_reg(alu_out_reg), 
        .fsm_state(fsm_state) // Exported state to help top-level muxing
        // Note: wb_addr_o is intentionally disconnected from FSM here, handled by top-level MUX below
    );

    ProgramCounter pc_unit (
        .clk(clk),
        .rst(rst),
        .pc_in((opcode == 7'b1101111 || opcode == 7'b1100111) ? alu_out_reg : alu_result), // Next PC always calculated by the ALU
        .pc_write(pc_write), 
        .pc_out(pc_out)
    );

    InstructionRegister ir (
        .clk(clk),
        .rst(rst),
        .ir_write(ir_write),
        .instruction_in(wb_dat_i), 
        .instruction_out(instruction)
    );

    InstructionDecoder decoder (
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .funct3(funct3),
        .funct7(funct7)
    );

    ALUControl alu_control_unit (
        .ALUOp(alu_op),
        .funct3(funct3),
        .funct7(funct7),
        .ALUControl(alu_ctrl_signal)
    );

    ImmediateGenerator imm_gen (
        .instruction(instruction),
        .immediate(immediate)
    );

    RegisterFile reg_file (
        .wb_clk(clk),
        .wb_rst(rst),
        .wb_we(register_write_final), // Safely gated to WRITEBACK state
        .write_addr(rd),
        .write_data(write_data),
        .read_addr1(rs1),
        .read_data1(read_data1),
        .read_addr2(rs2),
        .read_data2(read_data2)
    );

    ALU alu_unit (
        .operand_a(alu_src_final_a),
        .operand_b(alu_src_final_b),
        .ALUControl(alu_ctrl_signal),
        .result(alu_result),
        .z(alu_zero)
    );


    // ==========================================
    // 3. Datapath Multiplexers (Routing Logic)
    // ==========================================

    // ALU Source A MUX
    // If alu_src_a is 0: Use Register 1.
    // If alu_src_a is 1: If we are in FETCH (0), use current PC to do PC+4. 
    //                    Otherwise, use old_pc to calculate Branch targets.
    assign alu_src_final_a = (alu_src_a == 1'b0) ? read_data1 : 
                             (fsm_state == 3'd0) ? pc_out : old_pc;

    // ALU Source B MUX
    assign alu_src_final_b = (alu_src_b == 2'b00) ? read_data2 : 
                             (alu_src_b == 2'b01) ? immediate : 
                             32'd4; // 10: constant 4 for PC+4

    // Register Write Data MUX
    always @(*) begin
        if (opcode == 7'b1101111 || opcode == 7'b1100111) // JAL or JALR
            write_data = pc_out; // pc_out holds PC+4 at this stage
        `else if (opcode == 7'b0110111) // LUI
            write_data = immediate; // LUI writes the immediate value directly to the register
        else if (mem_to_reg)
            write_data = wb_dat_i; // Load from memory
        else
            write_data = alu_out_reg; // Stable ALU math result
    end

    // ==========================================
    // 4. External Wishbone Memory Routing
    // ==========================================
    
    // Data to be stored out to memory
    assign wb_dat_o = read_data2;

    // Wishbone Address MUX
    // If FETCH (state 0), request PC. Otherwise, request stable ALU Address.
    assign wb_adr_o = (fsm_state == 3'd0) ? pc_out : alu_out_reg;

endmodule