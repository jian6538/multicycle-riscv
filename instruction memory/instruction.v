// instr_mem.v
module instr_mem(

    input  wire        wb_clk_i,
    input  wire        wb_rst_i,

    // Wishbone Slave Interface
    input  wire        wb_cyc_i,    // Valid bus cycle
    input  wire        wb_stb_i,    // Valid data transfer
    input  wire        wb_we_i,     // Write Enable (0 = Read, 1 = Write)
    input  wire [31:0] wb_addr_i,   // Address in
    input  wire [31:0] wb_dat_i,    // Data in (if you ever write to it)
    
    output reg  [31:0] wb_dat_o,    // Data out (Instruction)
    output reg         wb_ack_o    // Acknowledge

);
  // Declare a memory array: 256 entries, each 32 bits wide.
  // This supports up to 256 instructions.
  reg [31:0] mem[0:255];

  // Pre-load the memory from a hex file at the start of the simulation.
  initial begin
    $readmemh("program.mem", mem);
  end

  // Asynchronous read. The 'addr' is shifted right by 2 because memory is
  // byte-addressed, but our array is word-addressed (32-bit words).
  always @(posedge wb_clk_i) begin
    if(wb_rst_i) begin
      wb_dat_o <= 32'b0; // Clear instruction on reset
      wb_ack_o <= 0;        // Clear acknowledge on reset
    end else begin
            wb_ack_o <= 0;        // Clear acknowledge by default

        if (wb_cyc_i && wb_stb_i && !wb_ack_o) begin
          wb_ack_o <= 1;        // Acknowledge the read
          if(!wb_we_i)begin
            wb_dat_o <= mem[wb_addr_i >> 2]; // Read instruction from memory
          
          end else begin
            mem[wb_addr_i >> 2] <= wb_dat_i; // Write to memory if wb_we_i is high
          end
      end
    end

  end

endmodule
