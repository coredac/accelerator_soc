`timescale 1ps/1ps
/*
vcs -sverilog -full64 -timescale=1ns/1ps ../erv_RISC-V/ALUCtrl.v ../erv_RISC-V/dff.v ../erv_RISC-V/immGen.v ../erv_RISC-V/mainCtrl.v ../erv_RISC-V/risc5ALU_core.v ../erv_RISC-V/riscV.v \
                                         ../common_cells/src/stream_mux.sv ../common_cells/src/stream_fork.sv ../common_cells/src/stream_fifo.sv \
                                         ../common_cells/src/stream_join.sv ../common_cells/src/stream_fork_dynamic.sv ../common_cells/src/fifo_v3.sv ../common_cells/src/stream_join_dynamic.sv \
                                         ../axi/src/axi_pkg.sv ../axi/src/axi_intf.sv ../axi/src/axi_to_detailed_mem.sv ../axi/src/axi_to_mem.sv \
                                         ../MeshMultiCgraRTL__explicit_vector_global_reduce__pickled.v \
                                         ../src_fir_vector/imem_rom.sv ../src_fir_vector/dp_sram_axi_cpu.sv ../src_fir_vector/cgra_axis_bridge.sv ../src_fir_vector/axis_dma_duplex.sv \
                                         ../src_fir_vector/acc_soc.sv ../verif_fir_vector/acc_soc_tb.sv \
                                         -debug_access+all +incdir+../common_cells/include +incdir+../axi/include +incdir+../erv_RISC-V -top acc_soc_tb
*/

module acc_soc_tb
(
);

  logic [0:0] clk;
  logic [0:0] reset;

  int PASS = 'd0;

  acc_soc dut ( .clk, .rstn(~reset) );

  localparam int N_PKTS   = 3;
  localparam longint SRC  = 64'h0000_0000_0000_1000; // must be inside SRAM range
  localparam longint DST  = 64'h0000_0000_0000_2000;

  initial
  begin
    // Static assertions
    if (CGRA_DATA_W != 67) $error("CGRA_DATA_W mismatch: %0d", CGRA_DATA_W);
    if (CGRA_CTRL_W != 107) $error("CGRA_CTRL_W mismatch: %0d", CGRA_CTRL_W);
    if (MCGR_PAY_W != 190) $error("MCGR_PAY_W mismatch: %0d", MCGR_PAY_W);
    if (PKT_HDR_W != 27) $error("PKT_HDR_W mismatch: %0d", PKT_HDR_W);
    if (CGRA_PKT_W != 217) $error("CGRA_PKT_W mismatch: %0d", CGRA_PKT_W);

    $display("\nTEST begin\n");

    clk = 1'b0;

    reset = 1'b0;
    #7
    reset = 1'b1;
    #50
    reset = 1'b0;
    #10
    repeat (10000) @(posedge clk);

    if ('d1 == PASS) $display("\nTEST PASSED.\n");
    else $display("\nTEST FAILED.\n");

    // Inspect the first few destination packets.
    for (int i=0;i<3;i++) begin
      $display("DST packet %0d: %02x %02x %02x %02x", i,
        dut.u_sram.mem[(64'h2000 + i*32 +  0) >> 3],
        dut.u_sram.mem[(64'h2000 + i*32 +  8) >> 3],
        dut.u_sram.mem[(64'h2000 + i*32 + 16) >> 3],
        dut.u_sram.mem[(64'h2000 + i*32 + 24) >> 3]);
    end

    $display("IMEM:");
    for (int b=0;b<30;b++)
      $display("%02x", dut.u_imem.mem[b]);

    $display("RF:");
    for (int b=0;b<32;b++)
      $display("%02x", dut.rf[b]);

    $display("\nTO CGRA");
    $display("SRAM[0x1000..0x1007]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1000 >> 3]);
    $display("SRAM[0x1008..0x100f]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1008 >> 3]);
    $display("SRAM[0x1010..0x1017]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1010 >> 3]);
    $display("SRAM[0x1018..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1018 >> 3]);

    $display("SRAM[0x1020..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1020 >> 3]);
    $display("SRAM[0x1028..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1028 >> 3]);
    $display("SRAM[0x1030..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1030 >> 3]);
    $display("SRAM[0x1038..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1038 >> 3]);

    $display("SRAM[0x1040..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1040 >> 3]);
    $display("SRAM[0x1048..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1048 >> 3]);
    $display("SRAM[0x1050..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1050 >> 3]);
    $display("SRAM[0x1058..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_1058 >> 3]);


    $display("\nFROM CGRA");
    $display("SRAM[0x2000..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2000 >> 3]);
    $display("SRAM[0x2008..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2008 >> 3]);
    $display("SRAM[0x2010..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2010 >> 3]);
    $display("SRAM[0x2018..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2018 >> 3]);

    $display("SRAM[0x2020..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2020 >> 3]);
    $display("SRAM[0x2028..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2028 >> 3]);
    $display("SRAM[0x2030..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2030 >> 3]);
    $display("SRAM[0x2038..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2038 >> 3]);

    $display("SRAM[0x2040..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2040 >> 3]);
    $display("SRAM[0x2048..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2048 >> 3]);
    $display("SRAM[0x2050..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2050 >> 3]);
    $display("SRAM[0x2058..]:");
    $display("%02x", dut.u_sram.mem[64'h0000_0000_0000_2058 >> 3]);


    $display("MMIO:");
    $display("%09b", {dut.reg_irq_en, 6'b0, dut.reg_start_tx, dut.reg_start_rx});
    $display("%02x %02x", dut.reg_src_rx, dut.reg_dst_tx);
    $display("%02x %02x", dut.reg_len_rx, dut.reg_len_tx);
    $display("%04b", {dut.stat_done_tx, dut.stat_busy_tx, dut.stat_done_rx, dut.stat_busy_rx});


    $finish();
  end

  initial
    forever
    begin
      #5
      clk = ~clk;
    end

  always @ (posedge clk or negedge clk)
  begin
    #1

    if ( ( ('h0000000000100000 == dut.u_sram.mem[64'h0000_0000_0000_2000 >> 3]) &&
           ('h9700238d10000000 == dut.u_sram.mem[64'h0000_0000_0000_2008 >> 3]) &&
           ('h1c00000000000022 == dut.u_sram.mem[64'h0000_0000_0000_2010 >> 3]) &&
           (         'h0180000 == dut.u_sram.mem[64'h0000_0000_0000_2018 >> 3]) ) &&
         ( ('h0000000000100000 == dut.u_sram.mem[64'h0000_0000_0000_2020 >> 3]) &&
           ('h9700238d10000000 == dut.u_sram.mem[64'h0000_0000_0000_2028 >> 3]) &&
           ('h1c00000000000022 == dut.u_sram.mem[64'h0000_0000_0000_2030 >> 3]) &&
           (         'h0182400 == dut.u_sram.mem[64'h0000_0000_0000_2038 >> 3]) ) )
    begin
      PASS = 1;
    end
  end

  /*initial
  begin  
    $dumpfile("./output.vcd");
    $dumpvars (0, cgra_test);
  end*/
`ifndef VERILATOR
  initial
  begin
    $fsdbDumpfile("./output.fsdb");
    //$fsdbDumpvars (0, "cgra_test.MultiCGRA.cgra__0.tile__5.send_data__msg");
    $fsdbDumpvars ("+all", "acc_soc_tb");
    $fsdbDumpMDA;
    $fsdbDumpSVA;
  end
`endif


endmodule

