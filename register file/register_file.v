// Register File Module for a 32-bit RISC-V Processor

module RegisterFile (
    input  wire        wb_clk,
    input  wire        wb_rst, // Although unused in this simple model, reset is good practice

    // Write Port
    input  wire        wb_we,      // Control signal to enable writing
    input  wire [4:0]  write_addr,      // Address of the register to write to (rd)
    input  wire [31:0] write_data,      // The data to write

    // Read Port 1
    input  wire [4:0]  read_addr1,      // Address of the first register to read (rs1)
    output wire [31:0] read_data1,      // Data read from the first port

    // Read Port 2
    input  wire [4:0]  read_addr2,      // Address of the second register to read (rs2)
    output wire [31:0] read_data2       // Data read from the second port
);

    // The core storage: 32 registers, each 32 bits wide.
    reg [31:0] registers [0:31];

    // --- Synchronous Write Logic ---
    // Writing only happens on the rising edge of the clock.
    always @(posedge wb_clk) begin
        // Check if the write signal is enabled AND the destination is not x0.
        if (wb_we && (write_addr != 5'b0)) begin
            registers[write_addr] <= write_data;
        end
    end

    // --- Asynchronous Read Logic ---
    // Reads are combinational; they happen immediately.
    // We check if the read address is x0. If so, output 0.
    // Otherwise, output the value from the register array.
    assign read_data1 = (read_addr1 == 5'b0) ? 32'b0 : registers[read_addr1];
    assign read_data2 = (read_addr2 == 5'b0) ? 32'b0 : registers[read_addr2];

endmodule
