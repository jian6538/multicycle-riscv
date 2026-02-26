# 1.Program Counter  
program counter is a special storage that store the next instruction address, the module accept the input and output the next instruction to instruction memory every cycle, normally the program counter is increase by 4 after complete
the previous instruction except for __branch instruction__ , __Jump Instructions__ , __Jump Register__ , __Exception or Interrupt__ and __Function Return__.

# 2.Instruction memory 
the instruction memory contain each the instruction and address, each instruction correspond to each address, the memory is store is byte , thus the instruction need to divide by 4 to correspond to each memory. 

| address        | instruction |
|----------      |----------|
| 32'h 00000000  | Cell 1   |
| 32'h 00000004  | Cell 2   |

| memory        | instruction |
|----------      |----------|
| 32'h 00000000  | Cell 1   |
| 32'h 00000001  | Cell 2   |

after fecting the instruction , the instruction is output to the InstructionDecoder and the ImmediateGenerator.

# 3. Instruction decoder 
instruction is a 32bit binary number, each range of the bit consist important information such as opcode , rd , rs1, rs2 , funct3 and funct7  
- since the opcode is not specific enough , opcode can only know what type of instruction needed to be perform(r type , i type..) , funct3 is used to distinguish
- the operation inside the opcode and funct 7 is further to distinguish the operation of funct3 (since some of funct3 consist of two operation)
- the opcode then is send to the control unit
- the rd , rs1 ,rs2 also send to register file

# 4. Immediate generator 
immediate generator is a special module to find the integer in the instruction since the each type of the instruction immediate is different we need to each case structure to extract the correct immediate as output.   
addi x5, x4, 10   
the immediate generator is to find out the number of 10 , in the binary instruction 

# 5. control unit 
the control unit is act as a brain, it receive the opcode from the instruction decoder and base on the opcode it decide the on and off of the switch ( RegWrite,
ALUSrc,
MemRead,
MemWrite,
MemToReg,
Branch,
ALUOp)  
those output will be send to alu , memory , register to further perform the process. 

# 6. register file 
the register file receive the address from the instruction decoder, that is register destination , register source 1 and register source 2. Register destination is the final value that need to write after perform the alu 
while the rs1 is source from instruction and rs2 is from the instruction as well but sometime the rs2 is a garbage from the instruction (for example due to the type of immediate instrution), thus we need a mux to select it. 
the register will also change the value of the address base on the result of alu and the need of change base on the control 

# 7. Mux 
the mux will receive two input, one is from the rs2 from the register file, it will be a garbage in immediate type case , and one is from the immediate generator, since the control unit will see the opcode and determine the on or off of the switch 
in this case the alusrc, the alusrc will determine whether to use the rs2 or the immediate. 

# 8. ALU control 
alu control is special component that look out the aluop from the control unit, since r type instruction has many operation such add , substract and so forth , thus the alu control will further see the funct3 and funct7 when it receive the r type aluop opcode 
and it will output the correct alu opcode to the alu 

# 9. Memory 
memory is just like a register but with more permanent, it will write or read the data base on the control output, it will read the data from the address or change the value in the memory base on the output of alu and the need of change 
# 10. ALU
finally , the alu will perform the operation base on the data receive from the rs1 and rs2(or immediate) base on the mux, and after the calculation it will output the final value 

