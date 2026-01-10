// -----------------------------------------------------------------------------
// Utility: 64-bit wide, dual-ported SRAM (CPU simple + PULP mem_req*)
// -----------------------------------------------------------------------------
module dp_sram_axi_cpu #(
  parameter longint DEPTH_WORD = 1024 * 64
)(
  input  logic        clk,
  input  logic        rstn,
  // Port A: CPU simple (byte addr)
  input  logic [63:0] cpu_addr,
  input  logic [63:0] cpu_wdata,
  input  logic        cpu_we,
  output logic [63:0] cpu_rdata,
  // Port B: PULP mem_req* (byte addr)
  input  logic        mem_req,
  output logic        mem_gnt,
  input  logic [31:0] mem_addr,
  input  logic [63:0] mem_wdata,
  input  logic [7:0]  mem_strb,
  input  logic        mem_we,
  output logic        mem_rvalid,
  output logic [63:0] mem_rdata
);
  // Storage
  logic [63:0] mem [0:DEPTH_WORD-1];

  logic [63:0] cpu_addr_aligned;
  logic [31:0] mem_addr_aligned;
  assign cpu_addr_aligned = cpu_addr >> 3;
  assign mem_addr_aligned = mem_addr >> 3;


  // Static loop indices (avoid automatic loop vars)
  int unsigned k_cpu_r, k_w, k_r;

  // ---------------- CPU read: purely combinational ----------------
  assign cpu_rdata = mem[cpu_addr_aligned];
  assign mem_rdata = mem[mem_addr_aligned];
  assign mem_rvalid = mem_req; //& ~mem_we; // rvalid is expected for both reads AND WRITES. Won't get bvalid out of axi_to_mem otherwise.
  // TODO: mem_rvalid =? mem_req & ( ~mem_we | (mem_we & ~cpu_we) )
  assign mem_gnt    = mem_req & ~cpu_we;
  // TODO: what is mem_gnt really? "..., request can be granted by this bank"?
  // ---------------- Unified writer + AXI read model ----------------
  // One always_ff drives: mem_gnt, mem_rvalid, mem_rdata, and *all* writes to mem
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin;
      //mem_gnt    <= 1'b0;
      //mem_rvalid <= 1'b0;
      //mem_rdata  <= '0;
    end else begin
      // Default handshakes
      //mem_gnt    <= mem_req & ~cpu_we;
      //mem_rvalid <= mem_req & ~mem_we;

      // AXI read data (one-cycle model)
      //if (mem_req && !mem_we) begin
      //  mem_rdata <= mem[mem_addr_aligned];
      //end

      // ---- Writes (two-writer block) ----
      // Priority: 1) CPU Port-A 2) AXI Port-B.
      if (cpu_we) begin
        mem[cpu_addr_aligned] <= cpu_wdata;
      end
      else if (mem_req && mem_we) begin
        for (int b = 0; b < 8; b++) begin
          mem[mem_addr_aligned][8*b +: 8] <= {8{mem_strb[b]}} & mem_wdata[8*b +: 8];
        end
      end
    end
  end

endmodule

