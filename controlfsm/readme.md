The Control Finite State Machine (FSM) acts as the brain of the multi-cycle processor. In this design, I integrated the state tracking and the control output signals into a single module.

The FSM relies on a state register that updates to the next state on every clock cycle. The transitions are determined by the Next State Logic (driven heavily by the opcode and the Wishbone mem_ack signal), while the Output Logic acts as the master control unit, sending the correct signals to the ALU, Register File, and Memory.

🔄 Next State Logic
FETCH: This state requests the next instruction from memory. The FSM will remain in this state until the memory asserts mem_ack (acknowledging the instruction is ready). Once mem_ack is high, it transitions to the DECODE state.

DECODE: The FSM decodes the instruction. Since it doesn't need to wait for external memory, it automatically transitions to EXECUTE on the next clock cycle.

EXECUTE: The next state depends entirely on the instruction's opcode:

If Load or Store: Transitions to the MEM state.

If R-Type, I-Type, U-Type, JAL, or JALR: Transitions to WRITEBACK.

If BEQ (Branch): It checks the ALU result. If the result is zero (values are equal), it transitions to BRANCH. If not zero, it returns to FETCH.

MEM: The FSM waits for the memory to finish reading or writing. It holds in this state until mem_ack is received. Once acknowledged, a Load instruction moves to WRITEBACK, while a Store instruction returns to FETCH.

WRITEBACK: The CPU writes the final computed value or loaded memory data back into the Register File. On the next clock cycle, it returns to FETCH.

BRANCH: The CPU calculates the branch target address. After one clock cycle, it returns to FETCH.

⚡ Output Logic
FETCH: The CPU asserts wb_cyc_o and wb_stb_o to request data from the instruction memory address. It also tells the ALU to calculate PC + 4. When mem_ack goes high, it asserts ir_write (to save the instruction) and pc_write (to update the PC to PC + 4).

DECODE: There are no specific active output signals required for the decode state.

EXECUTE: Output signals are routed based on the opcode. The FSM configures the ALU Source MUXes (alu_src_a and alu_src_b) and tells the ALU Control what mathematical or logical operation to perform.

MEM: The FSM checks if the instruction is a Store or Load, and asserts wb_we_o (Write Enable) accordingly to write or read data at the calculated memory address.

WRITEBACK: The FSM asserts RegWrite to write data to the destination register. For Jump instructions (JAL and JALR), it also asserts pc_write to jump to the new target address.

BRANCH: The FSM configures the ALU to add the Program Counter and the Immediate value, and asserts pc_write to update the PC to this new branch target.
