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
    wire        alu_src_a;        // Added missing declaration
    wire [1:0]  alu_src_b;        // Added missing declaration

    // Data path signals
    wire [31:0] read_data1, read_data2;
    wire [31:0] immediate;        
    wire [31:0] alu_result;
    wire        alu_zero;
    wire [31:0] write_data;
    
    // MUX output wires
    wire [31:0] alu_src_final_a;  // Added missing declaration
    wire [31:0] alu_src_final_b;  // Added missing declaration


    // --- Module Instantiations ---
    
    // 1. Control FSM
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
        .wb_addr_o(wb_adr_o),    
        .wb_we_o(wb_we_o),     
        .alu_src_a(alu_src_a),   
        .alu_src_b(alu_src_b),   
        .alu_op(alu_op),
        .MemRead(mem_read),
        .MemWrite(mem_write),
        .MemToReg(mem_to_reg),
        .Branch(branch),
        .RegWrite(reg_write),                  // Fixed case sensitivity
        .register_write_final(register_write_final) // Fixed case sensitivity & comma
    );

    // MUXes for ALU Inputs
    assign alu_src_final_a = (alu_src_a) ? pc_out : read_data1; 
    assign alu_src_final_b = (alu_src_b == 2'b00) ? read_data2 : 
                             (alu_src_b == 2'b01) ? immediate : 
                             32'd4; // 10: constant 4 for PC+4

    // 2. Program Counter
    // Assuming pc_unit uses PC_write as an enable signal, we just feed it alu_result.
    ProgramCounter pc_unit (
        .clk(clk),
        .rst(rst),
        .pc_in(alu_result), // Next PC always calculated by the ALU in a multicycle!
        .PC_write(pc_write), 
        .pc_out(pc_out)
    );

    // 3. Instruction Register (Holds wb_dat_i when ir_write is high)
    InstructionRegister ir (
        .clk(clk),
        .rst(rst),
        .ir_write(ir_write),
        .instruction_in(wb_dat_i), // Fetched from Wishbone bus
        .instruction_out(instruction)
    );

    // 4. Instruction Decoder
    InstructionDecoder decoder (
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .funct3(funct3),
        .funct7(funct7)
    );

    // 5. ALU Control
    ALUControl alu_control_unit (
        .ALUOp(alu_op),
        .funct3(funct3),
        .funct7(funct7),
        .ALUControl(alu_ctrl_signal)
    );

    // 6. Immediate Generator
    ImmediateGenerator imm_gen (
        .instruction(instruction),
        .immediate(immediate)
    );

    // 7. Register File
    RegisterFile reg_file (
        .clk(clk),
        .rst(rst),
        // Important: use register_write_final to ensure we only write in the WRITEBACK stage
        .write_enable(register_write_final), 
        .write_addr(rd),
        .write_data(write_data),
        .read_addr1(rs1),
        .read_data1(read_data1),
        .read_addr2(rs2),
        .read_data2(read_data2)
    );

    // 8. ALU
    ALU alu_unit (
        .operand_a(alu_src_final_a),
        .operand_b(alu_src_final_b),
        .ALUControl(alu_ctrl_signal),
        .result(alu_result),
        .z(alu_zero)
    );

    // --- External Memory Routing Logic ---
    
    // Route data to be stored out to the Wishbone bus
    assign wb_dat_o = read_data2;

    // MUX for write-back data to the Register File
    // If it's a load (mem_to_reg), take data from the Wishbone input bus. Otherwise, take ALU result.
    assign write_data = mem_to_reg ? wb_dat_i : alu_result;

endmodule