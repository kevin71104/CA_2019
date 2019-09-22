//========================================================================
// Lab 1 - Iterative Mul Unit
//========================================================================

`ifndef PARC_INT_MUL_ITERATIVE_V
`define PARC_INT_MUL_ITERATIVE_V

module imuldiv_IntMulIterative
(
  input         clk,
  input         reset,

  input  [31:0] mulreq_msg_a,
  input  [31:0] mulreq_msg_b,
  input         mulreq_val,
  output        mulreq_rdy,

  output [63:0] mulresp_msg_result,
  output        mulresp_val,
  input         mulresp_rdy
);
  wire   [ 5:0] state;
  wire   [ 1:0] state_nxt_sel, a_nxt_sel, b_nxt_sel, result_nxt_sel;

  imuldiv_IntMulIterativeDpath dpath
  (
    .clk                (clk),
    .reset              (reset),
    .mulreq_msg_a       (mulreq_msg_a),
    .mulreq_msg_b       (mulreq_msg_b),
    .mulreq_val         (mulreq_val),
    .mulreq_rdy         (mulreq_rdy),
    .mulresp_msg_result (mulresp_msg_result),
    .mulresp_val        (mulresp_val),
    .mulresp_rdy        (mulresp_rdy),
    .state              (state),
    .state_nxt_sel      (state_nxt_sel),
    .a_nxt_sel          (a_nxt_sel),
    .b_nxt_sel          (b_nxt_sel),
    .result_nxt_sel     (result_nxt_sel),
  );

  imuldiv_IntMulIterativeCtrl ctrl
  (
    .state              (state),
    .mulreq_val         (mulreq_val),
    .state_nxt_sel      (state_nxt_sel),
    .a_nxt_sel          (a_nxt_sel),
    .b_nxt_sel          (b_nxt_sel),
    .result_nxt_sel     (result_nxt_sel),
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntMulIterativeDpath
(
  input         clk,
  input         reset,

  input  [31:0] mulreq_msg_a,       // Operand A
  input  [31:0] mulreq_msg_b,       // Operand B
  input         mulreq_val,         // Request val Signal
  output        mulreq_rdy,         // Request rdy Signal

  output [63:0] mulresp_msg_result, // Result of operation
  output        mulresp_val,        // Response val Signal
  input         mulresp_rdy,        // Response rdy Signal

  output [ 5:0] state,
  input  [ 1:0] state_nxt_sel,
  input  [ 1:0] a_nxt_sel,
  input  [ 1:0] b_nxt_sel,
  input  [ 1:0] result_nxt_sel,
);
  //==== reg/wire declaration ===============================
  reg  [63:0] a_reg, a_reg_nxt;           // Register for storing operand A
  reg  [31:0] b_reg, b_reg_nxt;           // Register for storing operand B
  reg         sgn_reg, sgn_reg_nxt;       // Register for storing signed or unsigned result
  reg  [63:0] result_reg, result_reg_nxt; // Register for storing result
  reg  [ 5:0] state, state_nxt;           // FSM

  wire is_result_signed;
  wire sign_bit_a, sign_bit_b;
  wire [31:0] unsigned_a, unsigned_b;     //
  wire [63:0] a_shift;
  wire [31:0] b_shift;
  wire [63:0] alu_out;
  wire [ 5:0] state_p1;

  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  always @( posedge clk or posedge rst ) begin

    if (rst) begin              // Active high reset
      a_reg    <= 64'b0;
      b_reg    <= 32'b0;
      state    <=  6'b0;
      result_reg <= 64'b0;
      sgn_reg    <=  1'b0;
    end
    else if (mulresp_rdy) begin // Stall the pipeline if the response interface is not ready
      a_reg    <= a_reg_nxt;
      b_reg    <= b_reg_nxt;
      state    <= state_nxt;
      result_reg <= result_reg_nxt;
      sgn_reg  <= sgn_reg_nxt;    
    end

  end

  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // Extract sign bits
  assign sign_bit_a = a_reg[31];
  assign sign_bit_b = b_reg[31];
  assign is_result_signed = sign_bit_a ^ sign_bit_b;

  // Unsign operands if necessary
  assign unsigned_a = sign_bit_a ? (~a_reg + 1'b1) : a_reg;
  assign unsigned_b = sign_bit_b ? (~b_reg + 1'b1) : b_reg;

  //==== Computation logic ====

  // minor wires
  assign a_shift = a_reg << 1;
  assign b_shift = b_reg >> 1;
  assign alu_out = a_reg + result_reg;
  assign state_p1 = state + 6'd1;
  assign mulresp_msg_result
    = ( is_result_signed ) ? (~result_reg + 64'b1) : result_reg;  

  // MUX before Registers
  always@(*) begin
    if (state == 6'd0) begin // INIT STAGE: activate after receive req_val
      if (mulreq_val) begin
        a_reg_nxt = {32'b0, {unsigned_a}};
        b_reg_nxt = unsigned_b;
        state_nxt = state_p1;
      end
    end
    else if (state < 6'd33) begin // COMP STAGE: add 32 times
      a_reg_nxt = a_shift;
      b_reg_nxt = b_shift;
      state_nxt = state_p1;
      if (b_reg[0]) 
        result_reg_nxt = alu_out;
    end
  end

  // Set the val/rdy signals. The request is ready when the response is
  // ready, and the response is valid when there is valid data in the
  // input registers.

  //assign mulreq_rdy  = mulresp_rdy;
  //assign mulresp_val = val_reg;

endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntMulIterativeCtrl
(
  input [5:0] state,
  input       mulreq_val,
  output reg [1:0] state_nxt_sel,
  output reg [1:0] a_nxt_sel,
  output reg [1:0] b_nxt_sel,
  output reg [1:0] result_nxt_sel  //
);
  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // MUX selectors
  always@(*) begin
    // Default
    state_nxt_sel  = 1'b1;
    a_nxt_sel      = 1'b1;
    b_nxt_sel      = 1'b1;
    result_nxt_sel = 1'b0;

    if (state == 6'd0) begin // INIT STAGE: activate after receive req_val
      if (mulreq_val) begin
        a_nxt_sel     = 1'b1;
        b_nxt_sel     = 1'b1;
        state_nxt_sel = 1'b0;
        result_nxt_sel = 1'b1;
      end
    end
    else if (state < 6'd33) begin // COMP STAGE: add 32 times
      a_nxt_sel     = 1'b0;
      b_nxt_sel     = 1'b0;
      state_nxt_sel = 1'b0;
      result_nxt_sel = 1'b0;
    end
    else if (state == 6'd33) begin // COMP STAGE: add 32 times
      a_nxt_sel     = 1'b1;
      b_nxt_sel     = 1'b1;
      state_nxt_sel = 1'b1;
      result_nxt_sel = 1'b1;
    end
  end

endmodule

`endif
