// ============================================================================
// SoC (w/ PULP AXI) - BIDIRECTIONAL CGRA-to-Memory streaming.
//  - CPU (riscV): simple DMEM/IMEM ports.
//  - Shared RAM via PULP axi_to_mem -> dual-port SRAM (CPU simple + AXI).
//  - CGRA packet bridge (AXIS_W=192) to MeshMultiCgra wrapper.
//  - Duplex DMA:
//      * RX engine: Memory -> AXIS (-> CGRA recv_from_cpu).
//      * TX engine: AXIS (<- CGRA send_to_cpu) -> Memory.
//      Uses AXI bursts (3 beats / 24B per packet) on a 64-bit AXI bus.
//  - CPU MMIO @ 0x4000_0000 controls DMA (start/src/dst/len, status).
// ============================================================================
`timescale 1ns/1ps


// ---------- Bit widths ----------
localparam int CGRA_DATA_W = 64+1+1+1; // 67
localparam int CGRA_CTRL_W = 7 + 4*3 + 8*3 + 8*2 + 3 + 1 + 4*2 + 4*4 + 4*1 + 4*4; // 107
localparam int MCGR_PAY_W  = 5 + CGRA_DATA_W + 7 + CGRA_CTRL_W + 4; // 158
localparam int PKT_HDR_W   = 5+5+2+2+1+1+1+1+8+1; // 27
localparam int CGRA_PKT_W  = PKT_HDR_W + MCGR_PAY_W; // 217

localparam int BEATS_PER_PKT = (CGRA_PKT_W + AXI_DATA_W - 1) / AXI_DATA_W;


function automatic logic [CGRA_PKT_W-1:0] pack_pkt (IntraCgraPacket_4_2x2_16_8_2_CgraPayload__d294fd7ecd3c5b69 p);
  pack_pkt = {
    // Header (MSB->LSB order)
    p.src,
    p.dst,
    p.src_cgra_id,
    p.dst_cgra_id,
    p.src_cgra_x,
    p.src_cgra_y,
    p.dst_cgra_x,
    p.dst_cgra_y,
    p.opaque,
    p.vc_id,
    // Payload
    p.payload.cmd,
    p.payload.data.payload,
    p.payload.data.predicate,
    p.payload.data.bypass,
    p.payload.data.delay,
    p.payload.data_addr,
    p.payload.ctrl.operation,
    p.payload.ctrl.fu_in,
    p.payload.ctrl.routing_xbar_outport,
    p.payload.ctrl.fu_xbar_outport,
    p.payload.ctrl.vector_factor_power,
    p.payload.ctrl.is_last_ctrl,
    p.payload.ctrl.write_reg_from,
    p.payload.ctrl.write_reg_idx,
    p.payload.ctrl.read_reg_from,
    p.payload.ctrl.read_reg_idx,
    p.payload.ctrl_addr
  };
endfunction

function automatic IntraCgraPacket_4_2x2_16_8_2_CgraPayload__d294fd7ecd3c5b69 unpack_pkt (logic [CGRA_PKT_W-1:0] v);
  IntraCgraPacket_4_2x2_16_8_2_CgraPayload__d294fd7ecd3c5b69 p;
  // Use a running index from LSB upward for clarity
  int i = 0;
  // ctrl_addr (4)
  p.payload.ctrl_addr = v[i +: 4]; i += 4;
  // ctrl (107)
  p.payload.ctrl.read_reg_idx = v[i +: 16]; i += 16;
  p.payload.ctrl.read_reg_from = v[i +: 4]; i += 4;
  p.payload.ctrl.write_reg_idx = v[i +: 16]; i += 16;
  p.payload.ctrl.write_reg_from = v[i +: 8]; i += 8;
  p.payload.ctrl.is_last_ctrl = v[i +: 1]; i += 1;
  p.payload.ctrl.vector_factor_power = v[i +: 3]; i += 3;
  p.payload.ctrl.fu_xbar_outport = v[i +: 16]; i += 16; // 8*2
  p.payload.ctrl.routing_xbar_outport = v[i +: 24]; i += 24; // 8*3
  p.payload.ctrl.fu_in = v[i +: 12]; i += 12; // 4*3
  p.payload.ctrl.operation = v[i +: 7]; i += 7;
  // data_addr (7)
  p.payload.data_addr = v[i +: 7]; i += 7;
  // data (35)
  p.payload.data.delay = v[i +: 1]; i += 1;
  p.payload.data.bypass = v[i +: 1]; i += 1;
  p.payload.data.predicate = v[i +: 1]; i += 1;
  p.payload.data.payload = v[i +: 64]; i += 64;
  // cmd (5)
  p.payload.cmd = v[i +: 5]; i += 5;
  // header tail (27)
  p.vc_id = v[i +: 1]; i += 1;
  p.opaque = v[i +: 8]; i += 8;
  p.dst_cgra_y = v[i +: 1]; i += 1;
  p.dst_cgra_x = v[i +: 1]; i += 1;
  p.src_cgra_y = v[i +: 1]; i += 1;
  p.src_cgra_x = v[i +: 1]; i += 1;
  p.dst_cgra_id = v[i +: 2]; i += 2;
  p.src_cgra_id = v[i +: 2]; i += 2;
  p.dst = v[i +: 5]; i += 5;
  p.src = v[i +: 5]; i += 5;

  // Consistency check.
  if (i != CGRA_PKT_W)
    $error("unpack index mismatch: %0d != %0d", i, CGRA_PKT_W);

  return p;
endfunction


// -----------------------------------------------------------------------------
// DUT: SoC with MMIO @ 0x4000_0000 controlling the duplex DMA.
// -----------------------------------------------------------------------------
module acc_soc (
  input  logic clk,
  input  logic rstn
);
  // ---------------- CPU instance ----------------
  logic        [31:0] inst;
  logic        [63:0] PC_IMEM;
  logic        [63:0] readData_DMEM, addrData_DMEM, wrData_DMEM;
  logic               MemWrite_DMEM, MemRead_DMEM;
  logic signed [63:0] readData1_RF, readData2_RF; // Driven by RF.
  logic         [4:0] readAddr1_RF, readAddr2_RF, writeAddr_RF;
  logic        [63:0] writeData_RF;
  logic               RegWrite_RF;

  // CPU Imem ROM
  imem_rom u_imem( .pc(PC_IMEM[31:0]), .inst(inst) );

  // Register-file
  logic [63:0] rf [0:31];
  // Drive reads combinationally
  assign readData1_RF = (readAddr1_RF == 0) ? 64'd0 : rf[readAddr1_RF];
  assign readData2_RF = (readAddr2_RF == 0) ? 64'd0 : rf[readAddr2_RF];
  // Capture writes
  always_ff @(posedge clk) begin
    if (RegWrite_RF && (writeAddr_RF != 0))
      rf[writeAddr_RF] <= writeData_RF;
  end

  // CPU
  riscV u_cpu(
    .inst(inst), .PC_IMEM(PC_IMEM),
    .readData_DMEM(readData_DMEM), .addrData_DMEM(addrData_DMEM), .wrData_DMEM(wrData_DMEM),
    .MemWrite_DMEM(MemWrite_DMEM), .MemRead_DMEM(MemRead_DMEM),
    .readData1_RF(readData1_RF), .readData2_RF(readData2_RF),
    .readAddr1_RF(readAddr1_RF), .readAddr2_RF(readAddr2_RF),
    .writeData_RF(writeData_RF), .writeAddr_RF(writeAddr_RF), .RegWrite_RF(RegWrite_RF),
    .clk(clk), .rst(rstn)
  );

  // ---------------- MMIO registers ----------------
  localparam logic [63:0] MMIO_BASE = 64'h0000_0000_4000_0000;
  localparam logic [63:0] MMIO_MASK = 64'hFFFF_FFFF_FFFF_F000; // 4KB window
  logic is_mmio;
  assign is_mmio = (addrData_DMEM & MMIO_MASK) == (MMIO_BASE & MMIO_MASK);

  // Simple MMIO register map (all 64b unless noted):
  //  0x00 CONTROL   [0]=start_rx, [1]=start_tx, [8]=irq_en (optional)
  //  0x08 SRC_RX    64b byte addr (memory->CGRA)
  //  0x10 DST_TX    64b byte addr (CGRA->memory)
  //  0x18 LEN_RX    32b packet count
  //  0x1C LEN_TX    32b packet count
  //  0x20 STATUS    [0]=busy_rx, [1]=done_rx, [2]=busy_tx, [3]=done_tx
  logic        reg_start_rx, reg_start_tx, reg_irq_en;
  logic [63:0] reg_src_rx, reg_dst_tx;
  logic [31:0] reg_len_rx, reg_len_tx;
  logic        stat_busy_rx, stat_done_rx, stat_busy_tx, stat_done_tx;

  // Write
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      reg_start_rx<=0; reg_start_tx<=0; reg_irq_en<=0; reg_src_rx<='0; reg_dst_tx<='0; reg_len_rx<='0; reg_len_tx<='0;
    end else if (MemWrite_DMEM && is_mmio) begin
      unique case (addrData_DMEM[7:0])
        8'h00: begin reg_start_rx <= wrData_DMEM[0]; reg_start_tx <= wrData_DMEM[1]; reg_irq_en <= wrData_DMEM[8]; end
        8'h08: reg_src_rx <= wrData_DMEM;
        8'h10: reg_dst_tx <= wrData_DMEM;
        8'h18: reg_len_rx <= wrData_DMEM[31:0];
        8'h1C: reg_len_tx <= wrData_DMEM[31:0];
        default: ;
      endcase
    end else begin
      // auto-clear start strobes once DMA observes them (one-shot)
      if (reg_start_rx && stat_busy_rx) reg_start_rx <= 1'b0;
      if (reg_start_tx && stat_busy_tx) reg_start_tx <= 1'b0;
    end
  end

  // Read
  logic [63:0] mmio_rdata;
  always_comb begin
    unique case (addrData_DMEM[7:0])
      8'h00: mmio_rdata = {55'b0, reg_irq_en, 6'b0, reg_start_tx, reg_start_rx};
      8'h08: mmio_rdata = reg_src_rx;
      8'h10: mmio_rdata = reg_dst_tx;
      8'h18: mmio_rdata = {32'b0, reg_len_rx};
      8'h1C: mmio_rdata = {32'b0, reg_len_tx};
      8'h20: mmio_rdata = {60'b0, stat_done_tx, stat_busy_tx, stat_done_rx, stat_busy_rx};
      default: mmio_rdata = 64'b0;
    endcase
  end

  // Return path to CPU DMEM
  logic [63:0] ram_rdata;
  assign readData_DMEM = is_mmio ? mmio_rdata : ram_rdata;

  // ---------------- Dual-ported SRAM ----------------
  logic        mem_req, mem_gnt, mem_we, mem_rvalid;
  logic [31:0] mem_addr;
  logic [63:0] mem_wdata, mem_rdata;
  logic  [7:0] mem_strb;

  dp_sram_axi_cpu u_sram (
    .clk(clk), .rstn(rstn),
    // CPU simple port (DMEM)
    .cpu_addr (addrData_DMEM),
    .cpu_wdata(wrData_DMEM),
    .cpu_we   (MemWrite_DMEM & ~is_mmio),
    .cpu_rdata(ram_rdata),
    // AXI side
    .mem_req  (mem_req),
    .mem_gnt  (mem_gnt),
    .mem_addr (mem_addr),
    .mem_wdata(mem_wdata),
    .mem_strb (mem_strb),
    .mem_we   (mem_we),
    .mem_rvalid(mem_rvalid),
    .mem_rdata(mem_rdata)
  );

  // ---------------- PULP IP ----------------
  axi_req_t  dma_axi_req;
  axi_resp_t dma_axi_rsp;

  axi_to_mem #(
    .axi_req_t (axi_req_t),
    .axi_resp_t(axi_resp_t),
    .AddrWidth (AXI_ADDR_W),
    .DataWidth (AXI_DATA_W),
    .IdWidth   (AXI_ID_W),
    .NumBanks  (1),            // single bank -> simple SRAM
    .BufDepth  (BEATS_PER_PKT)
  ) u_axi2mem (
    .clk_i      (clk),
    .rst_ni     (rstn),
    .busy_o     (),
    .axi_req_i  (dma_axi_req),
    .axi_resp_o (dma_axi_rsp),
    .mem_req_o  (mem_req),
    .mem_gnt_i  (mem_gnt),
    .mem_addr_o (mem_addr),
    .mem_wdata_o(mem_wdata),
    .mem_strb_o (mem_strb),
    .mem_atop_o (),
    .mem_we_o   (mem_we),
    .mem_rvalid_i(mem_rvalid),
    .mem_rdata_i(mem_rdata)
  );

  // ---------------- Duplex DMA instance ----------------
  localparam int AXIS_W = AXI_DATA_W * ( (CGRA_PKT_W + AXI_DATA_W - 1) / AXI_DATA_W ) ; // 185 bits payload + 7 pad to get to a multiple of 64
  logic [AXIS_W-1:0] to_cgra_tdata;
  logic              to_cgra_tvalid, to_cgra_tready;
  logic [AXIS_W-1:0] from_cgra_tdata;
  logic              from_cgra_tvalid, from_cgra_tready;

  axis_dma_duplex #(
    .AXI_ADDR_W(AXI_ADDR_W), .AXI_DATA_W(AXI_DATA_W), .AXI_ID_W(AXI_ID_W), .AXIS_W(AXIS_W), .BEATS_PER_PKT(BEATS_PER_PKT)
  ) u_dma (
    .clk(clk), .rstn(rstn),
    .start_rx     (reg_start_rx),
    .src_addr_rx  (reg_src_rx),
    .len_pkts_rx  (reg_len_rx),
    .busy_rx      (stat_busy_rx),
    .done_rx      (stat_done_rx),
    .start_tx     (reg_start_tx),
    .dst_addr_tx  (reg_dst_tx),
    .len_pkts_tx  (reg_len_tx),
    .busy_tx      (stat_busy_tx),
    .done_tx      (stat_done_tx),
    .axi_i        (dma_axi_rsp),
    .axi_o        (dma_axi_req),
    .s_axis_tdata (to_cgra_tdata),
    .s_axis_tvalid(to_cgra_tvalid),
    .s_axis_tready(to_cgra_tready),
    .m_axis_tdata (from_cgra_tdata),
    .m_axis_tvalid(from_cgra_tvalid),
    .m_axis_tready(from_cgra_tready)
  );

  // ---------------- CGRA bridge (packet pack/unpack) ----------------
  // Import packet types and declare bridge<->CGRA wires
  //import cgra_pkt_pkg::*;
  IntraCgraPacket_4_2x2_16_8_2_CgraPayload__d294fd7ecd3c5b69 recv_msg, send_msg;
  logic [0:0] recv_val, recv_rdy;
  logic [0:0] send_val, send_rdy;

  // Bridge between AXIS and CGRA packet ports
  cgra_axis_bridge #(.AXIS_W(AXIS_W), .CGRA_PKT_W(CGRA_PKT_W)) u_cgra_bridge (
    .clk(clk), .rstn(rstn),
    .s_axis_tdata (to_cgra_tdata), .s_axis_tvalid(to_cgra_tvalid), .s_axis_tready(to_cgra_tready),
    .m_axis_tdata (from_cgra_tdata), .m_axis_tvalid(from_cgra_tvalid), .m_axis_tready(from_cgra_tready),
    .recv_from_cpu_pkt__msg(recv_msg), .recv_from_cpu_pkt__rdy(recv_rdy), .recv_from_cpu_pkt__val(recv_val),
    .send_to_cpu_pkt__msg  (send_msg), .send_to_cpu_pkt__rdy  (send_rdy), .send_to_cpu_pkt__val  (send_val)
  );

  // ---------------- CGRA instance wired to the bridge ----------------
  MeshMultiCgraRTL__explicit_vector_global_reduce u_cgra (
    .clk                    (clk),
    .reset                  (~rstn),
    .recv_from_cpu_pkt__msg (recv_msg),
    .recv_from_cpu_pkt__rdy (recv_rdy),
    .recv_from_cpu_pkt__val (recv_val),
    .send_to_cpu_pkt__msg   (send_msg),
    .send_to_cpu_pkt__rdy   (send_rdy),
    .send_to_cpu_pkt__val   (send_val)
  );
endmodule

