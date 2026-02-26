instruction memory is a read only memory and it can be read and cannot be modify. It read the instruction from the address given by the program counter.
for example 
a = b + c;
in assembly learning mean 
add x5, x6, x7
since assembly need to translate to machine code thus 
| funct7 | rs2 | rs1 | funct3 | rd | opcode |
|:---|:---|:---|:---|:---|:---|
| 0000000|00111|00110| 000 |00101|0110011|
| (for ADD)|(x7) |(x6) |(for ADD)|(x5) |(R-type)|

When you group these bits together, you get the final 32-bit binary instruction:
00000000011100110000001010110011

for the first 5 bit (2^5) rd. It tells the specific the register destination to store the data after completing the operation of alu.
for the another 5 bit (2^5) rs1. It tells the specific location of source 1. 
for the another 5 bit (2^5) rs2. It tells the specific location of source 2. 
as well as the immediate , which tell the processor to directly use that number 

risc v has different type of instruction normally the [6:0] is the opcode of the instruction and base on the opcode , the instruction is divided into several part like the image below 
![1_Mznpgo4kFWIayagpftLmTg](https://github.com/user-attachments/assets/ae5f28e3-15f2-445b-8ff8-ef462dc5237a)

for the opcode , the basic risc v has 6 instruction type which is register to register(R-type), immediate(i-type) , store (s-type) , branch(b-type) , upper immediate(u-type),jump (j-type) 


since the memory address is in word while the program counter is in byte , thus we need to divide 4 to know which word the memory are pointing 
