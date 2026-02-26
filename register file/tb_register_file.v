// Testbench for the RISC-V Register File module (tb_register_file.v)

`timescale 1ns / 1ps // Defines the simulation time units

module tb_register_file;

    // --- Testbench Signals ---
    // 'reg' types for inputs we will drive
    reg clk;
    reg rst;
    reg enable_write;
    reg [4:0] write_address;
    reg [31:0] write_data;
    reg [4:0] read_address;
    reg [4:0] read_address_two;

    // 'wire' types for outputs we will observe
    wire [31:0] read_data;
    wire [31:0] read_data_two;

    integer test_count = 0; // To keep track of tests

    // --- Instantiate the Device Under Test (DUT) ---
    // This connects our testbench signals to the register_file module.
    register_file dut (
        .clk(clk),
        .rst(rst),
        .enable_write(enable_write),
        .write_address(write_address),
        .write_data(write_data),
        .read_address(read_address),
        .read_address_two(read_address_two),
        .read_data(read_data),
        .read_data_two(read_data_two)
    );

    // --- Clock Generation ---
    // Create a clock signal with a 10ns period.
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // --- Main Test Sequence ---
    initial begin
        $display("Register File Testbench: Starting simulation.");

        // 1. Apply reset
        rst = 1;
        #15; // Hold reset for a bit
        rst = 0;
        #5;

        // --- Test 1: Basic Write and Read ---
        $display("\n--- Test 1: Basic Write and Read ---");
        enable_write = 1;
        write_address = 5; // Write to register x5
        write_data = 32'hABCDE123;
        @(posedge clk); // Wait for one clock cycle for the write to occur
        enable_write = 0; // Disable write for the read check
        
        read_address = 5;
        #1; // Allow combinational read logic to settle
        check_read("Read back from x5", read_data, 32'hABCDE123);


        // --- Test 2: Write Protection for x0 ---
        $display("\n--- Test 2: Attempt to write to x0 ---");
        enable_write = 1;
        write_address = 0; // Attempt to write to register x0
        write_data = 32'hDEADBEEF; // A non-zero value
        @(posedge clk);
        enable_write = 0;
        
        read_address = 0;
        #1;
        check_read("Read back from x0 after write attempt", read_data, 32'h00000000);

        // --- Test 3: Dual Asynchronous Read ---
        $display("\n--- Test 3: Simultaneous Dual Read ---");
        // Write two different values to two different registers
        enable_write = 1;
        write_address = 10; // Write to x10
        write_data = 32'd111;
        @(posedge clk);
        
        write_address = 20; // Write to x20
        write_data = 32'd222;
        @(posedge clk);
        enable_write = 0;

        // Now, read from both ports at the same time
        read_address = 10;
        read_address_two = 20;
        #1;
        check_read("Dual read port 1 (x10)", read_data, 32'd111);
        check_read("Dual read port 2 (x20)", read_data_two, 32'd222);


        $display("\nRegister File Testbench: All tests completed.");
        $finish; // End the simulation
    end

    // --- Self-checking Task for Read Verification ---
    task check_read;
        input [40*8-1:0] test_name; // A string to identify the test
        input [31:0] actual_data;
        input [31:0] expected_data;

        begin
            test_count = test_count + 1;
            if (actual_data === expected_data) begin
                $display("[%0d] PASS: %s", test_count, test_name);
            end else begin
                $display("[%0d] FAIL: %s", test_count, test_name);
                $display("       Expected: 0x%h", expected_data);
                $display("       Got:      0x%h", actual_data);
            end
        end
    endtask

endmodule

