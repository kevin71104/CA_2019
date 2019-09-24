//========================================================================
// Lab 1 - Iterative Div Unit
//========================================================================

`ifndef PARC_INT_DIV_ITERATIVE_V
`define PARC_INT_DIV_ITERATIVE_V

`include "imuldiv-DivReqMsg.v"

module imuldiv_IntDivIterative
(
  input         clk,
  input         reset,

  input         divreq_msg_fn,       // 1 : signed (IMULDIV_DIVREQ_MSG_FUNC_SIGNED)
  input  [31:0] divreq_msg_a,
  input  [31:0] divreq_msg_b,
  input         divreq_val,
  output        divreq_rdy,

  output [63:0] divresp_msg_result,
  output        divresp_val,
  input         divresp_rdy          //
);

  //==== reg/wire declaration ====
  wire   [ 5:0] state;
  wire   [ 1:0] state_nxt_sel, a_nxt_sel;
  wire          sub_mux_sel, div_sgn_nxt_sel, rem_sgn_nxt_sel;

  imuldiv_IntDivIterativeDpath dpath
  (
    .clk                (clk),
    .reset              (reset),
    .divreq_msg_fn      (divreq_msg_fn),
    .divreq_msg_a       (divreq_msg_a),
    .divreq_msg_b       (divreq_msg_b),
    .divreq_val         (divreq_val),
    .divreq_rdy         (divreq_rdy),
    .divresp_msg_result (divresp_msg_result),
    .divresp_val        (divresp_val),
    .divresp_rdy        (divresp_rdy),
    .state              (state),
    .state_nxt_sel      (state_nxt_sel),
    .a_nxt_sel          (a_nxt_sel),
    .sub_mux_sel        (sub_mux_sel),
    .div_sgn_nxt_sel    (div_sgn_nxt_sel),
    .sgn_nxt_sel        (rem_sgn_nxt_sel)
  );

  imuldiv_IntDivIterativeCtrl ctrl
  (
    .mulreq_val         (mulreq_val),
    .a_sign_bit         (divreq_msg_a[31]),
    .b_sign_bit         (divreq_msg_b[31]),
    .divreq_msg_fn      (divreq_msg_fn),
    .state              (state),
    .state_nxt_sel      (state_nxt_sel),
    .a_nxt_sel          (a_nxt_sel),
    .sub_mux_sel        (sub_mux_sel),
    .div_sgn_nxt_sel    (div_sgn_nxt_sel),
    .sgn_nxt_sel        (rem_sgn_nxt_sel)
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeDpath
(
  input         clk,
  input         reset,

  input         divreq_msg_fn,      // Function of MulDiv Unit
  input  [31:0] divreq_msg_a,       // Operand A
  input  [31:0] divreq_msg_b,       // Operand B
  input         divreq_val,         // Request val Signal
  output        divreq_rdy,         // Request rdy Signal

  output [63:0] divresp_msg_result, // Result of operation
  output        divresp_val,        // Response val Signal
  input         divresp_rdy,        // Response rdy Signal
  
  // Interface with Control Unit
  output [ 5:0] state,     
  input  [ 1:0] state_nxt_sel,
  input  [ 1:0] a_nxt_sel,
  input         sub_mux_sel,
  input         div_sgn_nxt_sel,
  input         rem_sgn_nxt_sel     //
);

  //==== reg/wire declaration ====
  reg  [63:0] a_reg;            // Register for storing operand A
  wire [63:0] a_reg_nxt;
  reg  [31:0] b_reg;            // Register for storing operand B
  wire [31:0] b_reg_nxt;
  reg         div_sgn_reg;      // Register for storing signed or unsigned quotient
  wire        div_sgn_reg_nxt;
  reg         rem_sgn_reg;      // Register for storing signed or unsigned remainder
  wire        rem_sgn_reg_nxt;
  reg  [ 5:0] state;            // FSM
  wire [ 5:0] state_nxt;
  reg         divresp_val;      // Output Reg
  wire        divresp_val_nxt; 
  reg         divreq_rdy;       // Output Reg
  wire        divreq_rdy_nxt;

  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // Extract sign bits

  wire sign_bit_a = a_reg[31];
  wire sign_bit_b = b_reg[31];

  // Unsign operands if necessary

  wire [31:0] unsigned_a = ( sign_bit_a ) ? (~a_reg + 1'b1) : a_reg;
  wire [31:0] unsigned_b = ( sign_bit_b ) ? (~b_reg + 1'b1) : b_reg;

  // Computation logic

  wire [31:0] unsigned_quotient
    = ( fn_reg == `IMULDIV_DIVREQ_MSG_FUNC_SIGNED )   ? unsigned_a / unsigned_b
    : ( fn_reg == `IMULDIV_DIVREQ_MSG_FUNC_UNSIGNED ) ? a_reg / b_reg
    :                                                   32'bx;

  wire [31:0] unsigned_remainder
    = ( fn_reg == `IMULDIV_DIVREQ_MSG_FUNC_SIGNED )   ? unsigned_a % unsigned_b
    : ( fn_reg == `IMULDIV_DIVREQ_MSG_FUNC_UNSIGNED ) ? a_reg % b_reg
    :                                                   32'bx;

  // Sign the final results if necessary

  wire [31:0] signed_quotient
    = ( fn_reg == `IMULDIV_DIVREQ_MSG_FUNC_SIGNED
     && is_result_signed_div ) ? ~unsigned_quotient + 1'b1
    :                            unsigned_quotient;

  wire [31:0] signed_remainder
    = ( fn_reg == `IMULDIV_DIVREQ_MSG_FUNC_SIGNED
     && is_result_signed_rem )   ? ~unsigned_remainder + 1'b1
   :                              unsigned_remainder;

  assign divresp_msg_result = { signed_remainder, signed_quotient };

  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  always @(posedge clk or posedge reset) begin
    if (reset) begin              // Active high reset
      
    end
    // Stall the pipeline if the response interface is not ready
    else if ( divresp_rdy ) begin
      a_reg   <= divreq_msg_a;
    end

  end

endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeCtrl
( 
  input            mulreq_val,
  input            a_sign_bit,
  input            b_sign_bit,
  input            divreq_msg_fn,

  input      [5:0] state,
  output reg [1:0] state_nxt_sel,
  output reg [1:0] a_nxt_sel,
  output reg       sub_mux_sel,
  output reg       div_sgn_nxt_sel,
  output reg       rem_sgn_nxt_sel     //
);

endmodule

`endif
