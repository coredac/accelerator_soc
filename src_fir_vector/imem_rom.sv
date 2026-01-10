// -----------------------------------------------------------------------------
// IMEM ROM (32b words, addressed by PC[31:2])
// -----------------------------------------------------------------------------
module imem_rom #(parameter int DEPTH = 4096) (
  input  logic [31:0] pc,
  output logic [31:0] inst
);
  logic [31:0] mem [0:DEPTH-1];
  //initial for (int i=0;i<DEPTH;i++) mem[i] = 32'h00000013; // ADDI x0,x0,0
  initial $readmemh("../sw/fir_vector_global_reduce.hex", mem);
  assign inst = mem[pc[31:2]];
endmodule

