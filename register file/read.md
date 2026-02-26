the register file contain 32 register and each register has 32 bit
imagine like the register is storing specific data instead of main memory , register file provide convenient to the alu by shorting the time. 

For example 
add x5, x6, x7
the file register need to read port x6 and x7 and write to x5. Thus , the register file has 2 read port and 1 write port. 

Why does not have 31 read port straight and 1 write port ?
because it is such a waste! the wiring will become very large thus cost a lot. 
