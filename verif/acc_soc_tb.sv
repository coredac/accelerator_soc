`timescale 1ps/1ps
/*
vcs -sverilog -full64 -timescale=1ns/1ps ../erv_RISC-V/ALUCtrl.v ../erv_RISC-V/dff.v ../erv_RISC-V/immGen.v ../erv_RISC-V/mainCtrl.v ../erv_RISC-V/risc5ALU_core.v ../erv_RISC-V/riscV.v \
                                         ../common_cells/src/stream_mux.sv ../common_cells/src/stream_fork.sv ../common_cells/src/stream_fifo.sv \
                                         ../common_cells/src/stream_join.sv ../common_cells/src/stream_fork_dynamic.sv ../common_cells/src/fifo_v3.sv ../common_cells/src/stream_join_dynamic.sv \
                                         ../axi/src/axi_pkg.sv ../axi/src/axi_intf.sv ../axi/src/axi_to_detailed_mem.sv ../axi/src/axi_to_mem.sv \
                                         ../MeshMultiCgraRTL__explicit__pickled.v \
                                         ../src/imem_rom.sv ../src/dp_sram_axi_cpu.sv ../src/cgra_axis_bridge.sv ../src/axis_dma_duplex.sv ../src/acc_soc.sv ../verif/acc_soc_tb.sv \
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

  int errors = 0;

  // Build a simple packet pattern
  function automatic IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed mk_pkt(int idx);
    IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed p;
    p.src         = 5'(idx);
    p.dst         = 5'(idx+1);
    p.src_cgra_id = 2'(1);
    p.dst_cgra_id = 2'(2);
    p.src_cgra_x  = 1'(0);
    p.src_cgra_y  = 1'(0);
    p.dst_cgra_x  = 1'(0);
    p.dst_cgra_y  = 1'(1);
    p.opaque      = 8'(8'hA0 + idx);
    p.vc_id       = 1'(0);
    p.payload.cmd = 5'(idx);
    p.payload.data.payload  = 32'(32'hDEAD_0000 + idx);
    p.payload.data.predicate= 1'(1);
    p.payload.data.bypass   = 1'(0);
    p.payload.data.delay    = 1'(0);
    p.payload.data_addr     = 7'(idx);
    p.payload.ctrl.operation= 7'(3);
    p.payload.ctrl.fu_in                = '{default:3'(0)};
    p.payload.ctrl.routing_xbar_outport = '{default:3'(0)};
    p.payload.ctrl.fu_xbar_outport      = '{default:2'(0)};
    p.payload.ctrl.vector_factor_power  = 3'(0);
    p.payload.ctrl.is_last_ctrl         = 1'(1);
    p.payload.ctrl.write_reg_from       = '{default:2'(0)};
    p.payload.ctrl.write_reg_idx        = '{default:4'(0)};
    p.payload.ctrl.read_reg_from        = '{default:1'(0)};
    p.payload.ctrl.read_reg_idx         = '{default:4'(0)};
    p.payload.ctrl_addr = 4'(idx);
    return p;
  endfunction

  // Helper to write 24-byte packet into DUT SRAM by hierarchical access
  task automatic sram_write24(longint addr, logic [191:0] d);
    // dp_sram is inside the top as u_sram, array is 'mem' (bytes)
    for (int b = 0; b < 24; b++) begin
      dut.u_sram.mem[addr + b] = d[b*8 +: 8];
    end
  endtask

  // Helper to read back 24 bytes from SRAM
  function automatic logic [191:0] sram_read24(longint addr);
    logic [191:0] out;
    for (int b = 0; b < 24; b++) begin
      out[b*8 +: 8] = dut.u_sram.mem[addr + b];
    end
    return out;
  endfunction

  // MMIO writer via direct register access (bypassing CPU)
  task automatic mmio_program_and_start(longint src, longint dst, int len_pkts);
    // Program the internal MMIO regs directly
    dut.reg_src_rx <= src;
    dut.reg_dst_tx <= dst;
    dut.reg_len_rx <= len_pkts;
    dut.reg_len_tx <= len_pkts;
    // one-shot start strobes
    dut.reg_start_rx <= 1'b1;
    dut.reg_start_tx <= 1'b1;
    @(posedge clk);
    dut.reg_start_rx <= 1'b0;
    dut.reg_start_tx <= 1'b0;
  endtask

  function automatic [31:0] enc_ADDI(input [4:0] rd, rs1, input integer imm);
    // opcode=0010011, funct3=000
    enc_ADDI = { {20{imm[11]}}, imm[11:0], rs1, 3'b000, rd, 7'b0010011 };
  endfunction

  function automatic [31:0] enc_SD (input [4:0] rs2, rs1, input integer imm);
    // S-type: opcode=0100011, funct3=011 (SD), imm[11:5], rs2, rs1, imm[4:0]
    enc_SD = { imm[11:5], rs2, rs1, 3'b011, imm[4:0], 7'b0100011 };
  endfunction


  initial
  begin
    // Static assertions
    if (CGRA_DATA_W != 35) $error("CGRA_DATA_W mismatch: %0d", CGRA_DATA_W);
    if (CGRA_CTRL_W != 107) $error("CGRA_CTRL_W mismatch: %0d", CGRA_CTRL_W);
    if (MCGR_PAY_W != 158) $error("MCGR_PAY_W mismatch: %0d", MCGR_PAY_W);
    if (PKT_HDR_W != 27) $error("PKT_HDR_W mismatch: %0d", PKT_HDR_W);
    if (CGRA_PKT_W != 185) $error("CGRA_PKT_W mismatch: %0d", CGRA_PKT_W);

    $display("\nTEST begin\n");

    clk = 1'b0;

    reset = 1'b0;
    #7
    reset = 1'b1;
    #50
    reset = 1'b0;
    #10
/*
    // 1) Preload N source packets in SRAM at SRC
    for (int i = 0; i < N_PKTS; i++) begin
      automatic IntraCgraPacket_4_2x2_16_8_2_CgraPayload__432fde8bfb7da0ed p = mk_pkt(i);
      automatic logic [CGRA_PKT_W-1:0] flat = pack_pkt(p);
      // place into 192b container (low bits used)
      logic [191:0] beat = '0;
      beat[CGRA_PKT_W-1:0] = flat;
      // TODO sram_write24(SRC + i*24, beat);
    end

    // 2) Program DMA MMIO and start both directions
    // TODO mmio_program_and_start(SRC, DST, N_PKTS);

    // 3) Wait for both done flags
    wait (dut.stat_done_rx && dut.stat_done_tx);

    // 4) Check destination equals source for each packet (echo behavior)
    for (int j = 0; j < N_PKTS; j++) begin
      automatic logic [191:0] src_pkt = sram_read24(SRC + j*24);
      automatic logic [191:0] dst_pkt = sram_read24(DST + j*24);
      if (dst_pkt !== src_pkt) begin
        $display("[ERR] Packet %0d mismatch: dst=%h src=%h", j, dst_pkt, src_pkt);
        errors++;
      end else begin
        $display("[OK ] Packet %0d matches: %h", j, dst_pkt);
      end
    end

    if (errors == 0) $display("[PASS] %0d packets moved mem->CGRA->mem correctly", N_PKTS);
    else             $display("[FAIL] %0d errors", errors);
*/
    repeat (2500) @(posedge clk);

    if ('d1 == PASS) $display("\nTEST PASSED.\n");
    else $display("\nTEST FAILED.\n");

    // Inspect the first few destination packets.
    for (int i=0;i<3;i++) begin
      $display("DST packet %0d: %02x %02x %02x", i,
        dut.u_sram.mem[(64'h2000 + i*24 +  0) >> 3],
        dut.u_sram.mem[(64'h2000 + i*24 +  8) >> 3],
        dut.u_sram.mem[(64'h2000 + i*24 + 16) >> 3]);
    end

    $display("IMEM:");
    for (int b=0;b<100;b++)
      $display("%02x", dut.u_imem.mem[b]);

    $display("ADDI %02x", enc_ADDI(6, 6, 'h456));
    $display("ADDI %02x", enc_ADDI(5, 5, 'h888));
    $display("SD   %02x", enc_SD(6, 1, 'h0));
    $display("SD   %02x", enc_SD(6, 6, 'h0));

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

    if ( (          'h100000 == dut.u_sram.mem[64'h0000_0000_0000_2000 >> 3]) &&
         ('h4f00238d10000000 == dut.u_sram.mem[64'h0000_0000_0000_2008 >> 3]) &&
         (  'h1800001c000011 == dut.u_sram.mem[64'h0000_0000_0000_2010 >> 3]) )
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

