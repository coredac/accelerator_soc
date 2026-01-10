// -----------------------------------------------------------------------------
// Duplex AXI-stream DMA (192b packet = 3×64b beats) with AXI bursts
// Control via simple wires (connected to MMIO regs in top)
// -----------------------------------------------------------------------------
`include "axi/typedef.svh"
`include "axi/assign.svh"
import axi_pkg::*;

// ---------------- PULP AXI types ----------------
localparam int AXI_ADDR_W    = 32;
localparam int AXI_DATA_W    = 64;
localparam int AXI_ID_W      = 4;
localparam int AXI_USER_W    = 1;

`AXI_TYPEDEF_ALL(axi,
  logic [AXI_ADDR_W-1:0],     // __addr_t
  logic [AXI_ID_W-1:0],       // __id_t
  logic [AXI_DATA_W-1:0],     // __data_t
  logic [(AXI_DATA_W/8)-1:0], // __strb_t
  logic [AXI_USER_W-1:0]      // __user_t
)


module axis_dma_duplex #(
  parameter int AXI_ADDR_W = 32,
  parameter int AXI_DATA_W = 64,
  parameter int AXI_ID_W   = 4,
  parameter int AXIS_W     = 192,
  parameter int BEATS_PER_PKT = (AXIS_W+AXI_DATA_W-1)/AXI_DATA_W // 192/64=3
)(
  input  logic clk,
  input  logic rstn,

  // Control/Status
  input  logic        start_rx,           // Memory->CGRA
  input  logic [63:0] src_addr_rx,
  input  logic [31:0] len_pkts_rx,        // number of 192b packets
  output logic        busy_rx,
  output logic        done_rx,

  input  logic        start_tx,           // CGRA->Memory
  input  logic [63:0] dst_addr_tx,
  input  logic [31:0] len_pkts_tx,
  output logic        busy_tx,
  output logic        done_tx,

  // AXI Master (PULP typed)
  input  axi_resp_t   axi_i,
  output axi_req_t    axi_o,

  // AXIS toward CGRA (ingress to CGRA)
  output logic [AXIS_W-1:0] s_axis_tdata,
  output logic              s_axis_tvalid,
  input  logic              s_axis_tready,

  // AXIS from CGRA (egress from CGRA)
  input  logic [AXIS_W-1:0] m_axis_tdata,
  input  logic              m_axis_tvalid,
  output logic              m_axis_tready
);

  // ---------------------------------------------------------------------------
  // AXI req defaulting
  // ---------------------------------------------------------------------------
  axi_req_t req_d, req_q;
  assign axi_o = req_q;
  always_comb begin
    req_d = '0;
    // defaults
    req_d.aw_valid = 1'b0; req_d.aw.id = '0; req_d.aw.len = BEATS_PER_PKT-1; req_d.aw.size = $clog2(AXI_DATA_W/8); req_d.aw.burst = BURST_INCR;
    req_d.w_valid  = 1'b0; req_d.w.strb = '1; req_d.w.last = 1'b0;
    req_d.b_ready  = 1'b1;
    req_d.ar_valid = 1'b0; req_d.ar.id = '0; req_d.ar.len = BEATS_PER_PKT-1; req_d.ar.size = $clog2(AXI_DATA_W/8); req_d.ar.burst = BURST_INCR;
    req_d.r_ready  = 1'b1;
  end

  // ---------------------------------------------------------------------------
  // RX engine: Memory -> AXIS
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {RX_IDLE, RX_AR, RX_R, RX_DONE} rx_state_e;
  rx_state_e rx_st; logic [31:0] rx_pkts_left; logic [AXI_ADDR_W-1:0] rx_addr;
  logic [2:0] rx_beat_cnt; logic [AXIS_W-1:0] rx_shift;

  assign busy_rx = (rx_st != RX_IDLE) && (rx_st != RX_DONE);
  //assign done_rx = (rx_st == RX_DONE);
  logic done_rx_nxt;
  assign done_rx_nxt = start_rx ? 'b0 : ( (rx_st == RX_DONE) ? 'b1 : done_rx );
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn)
      done_rx <= 'b0;
    else
      done_rx <= done_rx_nxt;
  end

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      rx_st <= RX_IDLE; rx_pkts_left <= '0; rx_addr <= '0; rx_beat_cnt <= '0; rx_shift <= '0;
      req_q.ar_valid <= '0;
      req_q.ar <= '0;
      req_q.r_ready <= '0;
    end else begin
      unique case (rx_st)
        RX_IDLE: if (start_rx) begin
          rx_pkts_left <= len_pkts_rx;
          rx_addr      <= src_addr_rx[AXI_ADDR_W-1:0];
          rx_st        <= RX_AR;
          req_q.r_ready <= 1'b0;
          req_q.ar_valid <= 1'b1; req_q.ar.addr <= src_addr_rx[AXI_ADDR_W-1:0]; req_q.ar.len <= BEATS_PER_PKT-1; req_q.ar.burst <= 'd1; req_q.ar.size <= $clog2(AXI_DATA_W/8);
        end
        RX_AR: begin
          // drive AR until accepted
          req_q.ar_valid <= 1'b1; req_q.ar.addr <= rx_addr; req_q.ar.len <= BEATS_PER_PKT-1; req_q.ar.burst <= 'd1; req_q.ar.size <= $clog2(AXI_DATA_W/8);
          if (axi_i.ar_ready) begin
            req_q.ar_valid <= 1'b0;
            rx_beat_cnt <= '0;
            rx_st <= RX_R;
            req_q.r_ready <= 1'b1;
          end
        end
        RX_R: begin
          // Assemble BEATS_PER_PKT beats into rx_shift[AXIS_W-1:0]
          if (axi_i.r_valid) begin
            // place each 64-bit beat into increasing chunks [63:0], [127:64], [191:128]
            rx_shift[rx_beat_cnt*AXI_DATA_W +: AXI_DATA_W] <= axi_i.r.data;
            if (rx_beat_cnt == BEATS_PER_PKT-1) begin
              // Have a full packet; offer to AXIS sink
              if (s_axis_tready) begin
                // Commit packet
                rx_beat_cnt <= '0;
                // Next packet
                if (rx_pkts_left > 1) begin
                  rx_pkts_left <= rx_pkts_left - 1;
                  rx_addr      <= rx_addr + BEATS_PER_PKT*(AXI_DATA_W/8);
                  rx_st        <= RX_AR;
                  req_q.r_ready <= 1'b0;
                  req_q.ar_valid <= 1'b1; req_q.ar.addr <= rx_addr + BEATS_PER_PKT*(AXI_DATA_W/8); req_q.ar.len <= BEATS_PER_PKT-1; req_q.ar.burst <= 'd1; req_q.ar.size <= $clog2(AXI_DATA_W/8);
                end else begin
                  rx_st <= RX_DONE;
                  req_q.r_ready <= 1'b0;
                end
              end
            end else begin
              rx_beat_cnt <= rx_beat_cnt + 1'b1;
            end
          end
        end
        RX_DONE: rx_st <= RX_IDLE;
        default: rx_st <= RX_IDLE;
      endcase
    end
  end

  // Present packet to AXIS sink (CGRA) when a full packet is buffered
  assign s_axis_tdata[(BEATS_PER_PKT-1)*AXI_DATA_W +: AXI_DATA_W] = axi_i.r.data;
  //assign s_axis_tdata[0 +: (BEATS_PER_PKT-1)*AXI_DATA_W]          = rx_shift[0 +: (BEATS_PER_PKT-1)*AXI_DATA_W];
  //assign s_axis_tdata = axi_i.r.data; TODO resilient fix for single beat per packet.
  genvar i;

  generate
    for (i = 0; i < (BEATS_PER_PKT-1); i = i + 1) begin : beat_i
      assign s_axis_tdata[i*AXI_DATA_W +: AXI_DATA_W] = rx_shift[i*AXI_DATA_W +: AXI_DATA_W];
    end
  endgenerate

  assign s_axis_tvalid = (rx_st == RX_R) && (rx_beat_cnt == BEATS_PER_PKT-1) && axi_i.r_valid;

  // ---------------------------------------------------------------------------
  // TX engine: AXIS -> Memory
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {TX_IDLE, TX_CAPTURE, TX_AW, TX_W, TX_B, TX_DONE} tx_state_e;
  tx_state_e tx_st; logic [31:0] tx_pkts_left; logic [AXI_ADDR_W-1:0] tx_addr;
  logic [1:0] tx_beat_cnt; logic [AXIS_W-1:0] tx_buf;

  assign busy_tx = (tx_st != TX_IDLE) && (tx_st != TX_DONE);
  //assign done_tx = (tx_st == TX_DONE);
  logic done_tx_nxt;
  assign done_tx_nxt = start_tx ? 'b0 : ( (tx_st == TX_DONE) ? 'b1 : done_tx );
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn)
      done_tx <= 'b0;
    else
      done_tx <= done_tx_nxt;
  end

  // Always ready to accept a packet when in CAPTURE
  assign m_axis_tready = (tx_st == TX_CAPTURE);

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      tx_st <= TX_IDLE; tx_pkts_left <= '0; tx_addr <= '0; tx_beat_cnt <= '0; tx_buf <= '0;
      req_q.aw_valid <= 1'b0;
      req_q.aw <= '0;
      req_q.w <= '0;
      req_q.w_valid <= 1'b0;
      req_q.b_ready <= 1'b0;
    end else begin
      unique case (tx_st)
        TX_IDLE: if (start_tx) begin
          tx_pkts_left <= len_pkts_tx;
          tx_addr      <= dst_addr_tx[AXI_ADDR_W-1:0];
          tx_st        <= TX_CAPTURE;
        end
        TX_CAPTURE: begin
          if (m_axis_tvalid) begin
            tx_buf <= m_axis_tdata;
            tx_st  <= TX_AW;
            tx_beat_cnt <= '0;
            //req_q.aw_valid <= 1'b1; req_q.aw.addr <= tx_addr; req_q.aw.len <= BEATS_PER_PKT-1; req_q.aw.burst <= 'd1; req_q.aw.size <= $clog2(AXI_DATA_W/8);
            //req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[tx_beat_cnt*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last <= (tx_beat_cnt == BEATS_PER_PKT-1);
          end
        end
        TX_AW: begin
          req_q.aw_valid <= 1'b1; req_q.aw.addr <= tx_addr; req_q.aw.len <= BEATS_PER_PKT-1; req_q.aw.burst <= 'd1; req_q.aw.size <= $clog2(AXI_DATA_W/8);
          req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[tx_beat_cnt*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last <= (tx_beat_cnt == BEATS_PER_PKT-1);
          req_q.b_ready <= 1'b1;
          if (axi_i.aw_ready && (tx_beat_cnt != BEATS_PER_PKT-1) ) begin // 1 != BEATS_PER_PKT
            req_q.aw_valid <= 1'b0;
            tx_beat_cnt <= 'b1;
            tx_st <= TX_W;
            req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[1*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last <= (1 == BEATS_PER_PKT-1);
          end else if (axi_i.aw_ready && (tx_beat_cnt == BEATS_PER_PKT-1) && ~axi_i.b_valid) begin // 1 == BEATS_PER_PKT
            tx_st <= TX_B;
            req_q.w_valid <= 1'b0;
            req_q.aw_valid <= 1'b0;
          end else if (axi_i.aw_ready && (tx_beat_cnt == BEATS_PER_PKT-1) && axi_i.b_valid) begin // 1 == BEATS_PER_PKT
            //tx_st <= TX_DONE;
            req_q.w_valid <= 1'b0;
            req_q.aw_valid <= 1'b0;
            if (tx_pkts_left > 1) begin
              tx_pkts_left <= tx_pkts_left - 1;
              tx_addr      <= tx_addr + BEATS_PER_PKT*(AXI_DATA_W/8);
              tx_st        <= TX_CAPTURE;
            end else begin
              tx_st <= TX_DONE;
            end
          end
        end
        TX_W: begin
          //req_q.aw_valid <= 1'b1; req_q.aw.addr <= tx_addr; req_q.aw.len <= BEATS_PER_PKT-1; req_q.aw.burst <= 'd1; req_q.aw.size <= $clog2(AXI_DATA_W/8);
          // drive each 64-bit chunk sequentially
          req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[tx_beat_cnt*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last  <= (tx_beat_cnt == BEATS_PER_PKT-1);
          req_q.b_ready <= 1'b1;
          if (axi_i.w_ready) begin
            if ( (tx_beat_cnt == BEATS_PER_PKT-1) && ~axi_i.b_valid ) begin
              req_q.b_ready <= 1'b1;
              tx_st <= TX_B;
              req_q.w_valid <= 1'b0;
            end else if ( (tx_beat_cnt == BEATS_PER_PKT-1) && axi_i.b_valid ) begin
              //req_q.b_ready <= 1'b1;
              //tx_st <= TX_DONE;
              req_q.w_valid <= 1'b0;
              if (tx_pkts_left > 1) begin
                tx_pkts_left <= tx_pkts_left - 1;
                tx_addr      <= tx_addr + BEATS_PER_PKT*(AXI_DATA_W/8);
                tx_st        <= TX_CAPTURE;
              end else begin
                tx_st <= TX_DONE;
              end
            end else begin
              tx_beat_cnt <= tx_beat_cnt + 1'b1;
              req_q.w_valid <= 1'b1; req_q.w.data <= tx_buf[(tx_beat_cnt+1'b1)*AXI_DATA_W +: AXI_DATA_W]; req_q.w.strb <= {8{1'b1}}; req_q.w.last  <= ((tx_beat_cnt+1'b1) == BEATS_PER_PKT-1);
            end
          end
        end
        TX_B: begin
          if (axi_i.b_valid) begin
            // Next packet or done
            req_q.b_ready <= 1'b0;
            if (tx_pkts_left > 1) begin
              tx_pkts_left <= tx_pkts_left - 1;
              tx_addr      <= tx_addr + BEATS_PER_PKT*(AXI_DATA_W/8);
              tx_st        <= TX_CAPTURE;
            end else begin
              tx_st <= TX_DONE;
            end
          end
        end
        TX_DONE: tx_st <= TX_IDLE;
        default: tx_st <= TX_IDLE;
      endcase
    end
  end
endmodule

