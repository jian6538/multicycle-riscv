This project is a continuation of my journey learning RISC-V architecture. Originally, I wanted to integrate a biquad filter with my single-cycle RISC-V core. However, the single-cycle design lacks a Wishbone bus protocol and assumes data updates immediately. By introducing an acknowledgment signal, the CPU now knows exactly when data is ready.

To support this, this multi-cycle RISC-V introduces a Control Finite State Machine (FSM) to manage the flow of the FETCH, DECODE, EXECUTE, MEM, WRITEBACK, and BRANCH states. Unlike a 5-stage pipelined RISC-V, this multi-cycle module only performs one task per clock cycle. (I will be building a 5-stage pipeline in a future repository!).

Through this project, I learned the basic architecture of the Wishbone bus protocol, as well as how to design a datapath and control unit by introducing registers for each stage.
