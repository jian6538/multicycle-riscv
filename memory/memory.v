// Data Memory Module for a 32-bit RISC-V Processor

module DataMemory (
    input  wire        wb_clk,
    input wire wb_cyc_i,    // Valid bus cycle
    input wire wb_stb_i,    // Valid data transfer
    input wire wb_we_i,     // Write Enable (0 = Read, 1 = Write)
    input wire [31:0] wb_addr_i,   // Address in
    input wire [31:0] wb_dat_i,    // Data in (if you


    // Output
    output reg wb_ack,
    output reg [31:0] wb_dat_o       // Data that was read from memory
);

    // Main storage: 1024 slots, each 32 bits wide.
    reg [31:0] memory [0:1023];

    // --- Synchronous Write Logic ---
    // Writing to memory only happens on the rising edge of the clock
    // to ensure stability.
    always @(posedge wb_clk) begin
        wb_ack <= 0; // Clear acknowledge by default
        // If the memWrite signal is high, perform the write.
        if (!wb_ack && wb_stb_i && wb_cyc_i) begin
            wb_ack <= 1; // Set acknowledge when write is performed
            if(wb_we_i) begin
                // Convert the byte address to a word index for our memory array (divide by 4).
                memory[wb_addr_i[11:2]] <= wb_dat_i;
            end else if(!wb_we_i) begin
                // For reads, we will set the output data in the next clock cycle.
                // The Control Unit will decide whether to actually *use* this data or ignore it.
                wb_dat_o <= memory[wb_addr_i[11:2]];
            end
             // Convert the byte address to a word index for our memory array (divide by 4
        end

    end
    


endmodule
