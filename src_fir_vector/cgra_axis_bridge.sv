// -----------------------------------------------------------------------------
// Import the CGRA packet bridge (pack/unpack) — assumed present in your tree
//   package cgra_pkt_pkg
//   module cgra_axis_bridge #(AXIS_W=192)
// -----------------------------------------------------------------------------
// ============================================================
// 1-beat AXI-Stream bridge to MeshMultiCgraRTL ports
// - s_axis_* : DMA -> CGRA (CPU->CGRA direction)
// - m_axis_* : CGRA -> DMA (CGRA->CPU direction)
// ============================================================
module cgra_axis_bridge #(
  parameter int AXIS_W     = 192, // must be >= cgra_pkt_pkg::CGRA_PKT_W (185)
  parameter int CGRA_PKT_W = 185
)(
  input logic clk,
  input logic rstn,

  // ------------------ AXI-Stream to CGRA (ingress) ------------------
  input  logic [AXIS_W-1:0] s_axis_tdata,
  input  logic              s_axis_tvalid,
  output logic              s_axis_tready,

  // ------------------ AXI-Stream from CGRA (egress) -----------------
  output logic [AXIS_W-1:0] m_axis_tdata,
  output logic              m_axis_tvalid,
  input  logic              m_axis_tready,

  // ------------------ CGRA ports -----------------------------
  output IntraCgraPacket_4_2x2_16_8_2_CgraPayload__d294fd7ecd3c5b69 recv_from_cpu_pkt__msg,
  input  logic [0:0]                                                recv_from_cpu_pkt__rdy,
  output logic [0:0]                                                recv_from_cpu_pkt__val,

  input  IntraCgraPacket_4_2x2_16_8_2_CgraPayload__d294fd7ecd3c5b69 send_to_cpu_pkt__msg,
  output logic [0:0]                                                send_to_cpu_pkt__rdy,
  input  logic [0:0]                                                send_to_cpu_pkt__val
);
  //import cgra_pkt_pkg::*;
  // Pad/trim helpers
  localparam int PKT_W = CGRA_PKT_W; // 185
  localparam int PAD_W = (AXIS_W >= PKT_W) ? (AXIS_W - PKT_W) : -1;
  initial if (AXIS_W < PKT_W) $error("AXIS_W (%0d) < CGRA packet width (%0d)", AXIS_W, PKT_W);

  // ---------- Ingress: AXIS -> CGRA ----------
  // 1-beat transfer: when both s_axis_tvalid && recv_from_cpu_pkt__rdy.
  assign s_axis_tready                = recv_from_cpu_pkt__rdy;
  assign recv_from_cpu_pkt__val       = s_axis_tvalid;
  assign recv_from_cpu_pkt__msg       = unpack_pkt(s_axis_tdata[PKT_W-1:0]);

  assign send_to_cpu_pkt__rdy         = m_axis_tready;
  assign m_axis_tvalid                = send_to_cpu_pkt__val;
  assign m_axis_tdata[PKT_W-1:0]      = pack_pkt(send_to_cpu_pkt__msg);
  assign m_axis_tdata[AXIS_W-1:PKT_W] = '0;

  /*
  // ============================================================================
  // Ingress path: AXIS -> (buffer) -> CGRA.recv_from_cpu_pkt
  // ============================================================================
  // One-entry skid buffer to avoid combinational loops and ease timing
  logic               in_full;
  logic [AXIS_W-1:0]  in_data_q;

  // Accept from AXIS when buffer not full
  assign s_axis_tready = ~in_full;

  // Buffer fill / drain
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      in_full   <= 1'b0;
      in_data_q <= '0;
    end else begin
      // Fill on handshake if empty
      if (!in_full && s_axis_tvalid && s_axis_tready) begin
        in_data_q <= s_axis_tdata;
        in_full   <= 1'b1;
      end
      // Drain to CGRA when it takes it
      if (in_full && recv_from_cpu_pkt__rdy[0]) begin
        in_full <= 1'b0;
      end
    end
  end

  // Drive CGRA typed message/val from buffer
  assign recv_from_cpu_pkt__val = in_full;
  assign recv_from_cpu_pkt__msg = unpack_pkt(in_data_q[PKT_W-1:0]);

  // ============================================================================
  // Egress path: CGRA.send_to_cpu_pkt -> (buffer) -> AXIS
  // ============================================================================
  logic               out_full;
  logic [AXIS_W-1:0]  out_data_q;

  // Backpressure toward CGRA (ready only when buffer has space)
  assign send_to_cpu_pkt__rdy = {~out_full};

  // Capture from CGRA when it produces and buffer empty
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      out_full   <= 1'b0;
      out_data_q <= '0;
    end else begin
      if (!out_full && send_to_cpu_pkt__val[0] && send_to_cpu_pkt__rdy[0]) begin
        out_data_q                <= '0;
        out_data_q[PKT_W-1:0]     <= pack_pkt(send_to_cpu_pkt__msg);
        out_full                  <= 1'b1;
      end
      // AXIS consumer handshake drains the buffer
      if (out_full && m_axis_tready) begin
        out_full <= 1'b0;
      end
    end
  end

  assign m_axis_tvalid = out_full;
  assign m_axis_tdata  = out_data_q;
  */
endmodule

