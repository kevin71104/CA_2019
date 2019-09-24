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
  wire          b_nxt_sel, div_sgn_nxt_sel, rem_sgn_nxt_sel;
  wire          div_sgn, rem_sgn;

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
    .div_sgn            (div_sgn),
    .rem_sgn            (rem_sgn),
    .state_nxt_sel      (state_nxt_sel),
    .a_nxt_sel          (a_nxt_sel),
    .b_nxt_sel          (b_nxt_sel),
    .div_sgn_nxt_sel    (div_sgn_nxt_sel),
    .rem_sgn_nxt_sel    (rem_sgn_nxt_sel)
  );

  imuldiv_IntDivIterativeCtrl ctrl
  (
    .divreq_val         (divreq_val),
    .a_sign_bit         (divreq_msg_a[31]),
    .b_sign_bit         (divreq_msg_b[31]),
    .divreq_msg_fn      (divreq_msg_fn),
    .state              (state),
    .div_sgn            (div_sgn),
    .rem_sgn            (rem_sgn),
    .state_nxt_sel      (state_nxt_sel),
    .a_nxt_sel          (a_nxt_sel),
    .b_nxt_sel          (b_nxt_sel),
    .div_sgn_nxt_sel    (div_sgn_nxt_sel),
    .rem_sgn_nxt_sel    (rem_sgn_nxt_sel)
  );

endmodule

//------------------------------------------------------------------------
// Datapath
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeDpath
(
  input         clk,
  input         reset,

  input         divreq_msg_fn,      // 1 : signed (IMULDIV_DIVREQ_MSG_FUNC_SIGNED)
  input  [31:0] divreq_msg_a,       // Operand A
  input  [31:0] divreq_msg_b,       // Operand B
  input         divreq_val,         // Request val Signal
  output        divreq_rdy,         // Request rdy Signal

  output [63:0] divresp_msg_result, // Result of operation
  output        divresp_val,        // Response val Signal
  input         divresp_rdy,        // Response rdy Signal
  
  // Interface with Control Unit
  output [ 5:0] state,
  input         div_sgn,
  input         rem_sgn,
  input  [ 1:0] state_nxt_sel,
  input  [ 1:0] a_nxt_sel,
  input         b_nxt_sel,
  input         div_sgn_nxt_sel,
  input         rem_sgn_nxt_sel     //
);

  //==== reg/wire declaration ====
  reg  [64:0] a_reg;            // Register for storing operand A
  wire [64:0] a_reg_nxt;
  reg  [64:0] b_reg;            // Register for storing operand B
  wire [64:0] b_reg_nxt;
  reg         div_sgn_reg;      // Register for storing signed or unsigned quotient
  wire        div_sgn_reg_nxt;
  reg         rem_sgn_reg;      // Register for storing signed or unsigned remainder
  wire        rem_sgn_reg_nxt;
  reg  [ 5:0] state;            // FSM
  wire [ 5:0] state_nxt;
  //reg         divresp_val;      // Output Reg
  //wire        divresp_val_nxt; 
  //reg         divreq_rdy;       // Output Reg
  //wire        divreq_rdy_nxt;

  wire        sign_bit_a;
  wire        sign_bit_b;
  wire        is_oper_signed;
  wire [31:0] unsigned_a;
  wire [31:0] unsigned_b;
  wire [64:0] a_init;
  wire [64:0] b_init;
  wire [64:0] a_shift;
  wire [64:0] sub_out, sub_mux_out;
  wire [ 5:0] state_p1;
  wire [31:0] f_result_div, f_result_rem;

  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------
  wire [31:0] div_tmp = a_reg[31:0];
  wire [31:0] rem_tmp = a_reg[63:32];

  // Extract sign bits and Unsign operands if necessary
  assign sign_bit_a     = divreq_msg_a[31];
  assign sign_bit_b     = divreq_msg_b[31];
  assign is_oper_signed = (divreq_msg_fn == `IMULDIV_DIVREQ_MSG_FUNC_SIGNED);
  assign unsigned_a     = (sign_bit_a & is_oper_signed) ? (~divreq_msg_a + 1'b1) : divreq_msg_a;
  assign unsigned_b     = (sign_bit_b & is_oper_signed) ? (~divreq_msg_b + 1'b1) : divreq_msg_b;

  // minor wires
  assign a_init   = {33'b0, {unsigned_a}};
  assign b_init   = {1'b0, {unsigned_b}, 32'b0};
  assign a_shift  = a_reg << 1;
  assign state_p1 = state + 6'd1;
  assign sub_out  = a_shift - b_reg;

  // MUX
  assign sub_mux_out  = sub_out[64] ? a_shift : {{sub_out[64:1]}, 1'b1};
  assign f_result_div = div_sgn_reg ? (~sub_mux_out[31:0]  + 32'b1) : sub_mux_out[31:0];
  assign f_result_rem = rem_sgn_reg ? (~sub_mux_out[63:32] + 32'b1) : sub_mux_out[63:32];

  // MUX before Registers: Option3 -> Option2 -> Option1 -> Option0
  assign a_reg_nxt       = a_nxt_sel[1] ?
                           (a_nxt_sel[0] ? {1'b0, {f_result_rem}, {f_result_div}} : sub_mux_out) :
                           (a_nxt_sel[0] ? a_init : a_reg);
  assign b_reg_nxt       = b_nxt_sel ? b_init : b_reg;
  assign div_sgn_reg_nxt = div_sgn_nxt_sel ? div_sgn : div_sgn_reg;
  assign rem_sgn_reg_nxt = rem_sgn_nxt_sel ? rem_sgn : rem_sgn_reg;
  assign state_nxt       = state_nxt_sel[1] ? state_p1 :
                           state_nxt_sel[0] ? 6'b0 : state;

  //==== OUTPUT SECTION ====
  assign divresp_msg_result = a_reg[63:0];
  assign divreq_rdy  = reset || (state == 6'd0 && ~divreq_val) || (state == 6'd33);
  assign divresp_val = state == 6'd33;
  //assign divreq_rdy_nxt     = divresp_rdy && ((state == 6'd32) || (state == 6'd0 && ~divreq_val));
  //assign divresp_val_nxt    = divresp_rdy && (state == 6'd32);

  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  always @(posedge clk or posedge reset) begin
    if (reset) begin                   // Active high reset
      a_reg       <= 65'b0;
      b_reg       <= 65'b0;
      div_sgn_reg <=  1'b0;
      rem_sgn_reg <=  1'b0;
      state       <=  6'b0;
      //divresp_val <=  1'b0;
      //divreq_rdy  <=  1'b1;
    end
    else if (divresp_rdy) begin        // Stall the pipeline if the response interface is not ready
      a_reg       <= a_reg_nxt;
      b_reg       <= b_reg_nxt;
      div_sgn_reg <= div_sgn_reg_nxt;
      rem_sgn_reg <= rem_sgn_reg_nxt;
      state       <= state_nxt;
      //divresp_val <= divresp_val_nxt;
      //divreq_rdy  <= divreq_rdy_nxt;
    end

  end

endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntDivIterativeCtrl
( 
  input            divreq_val,
  input            a_sign_bit,
  input            b_sign_bit,
  input            divreq_msg_fn,

  input      [5:0] state,
  output           div_sgn,
  output           rem_sgn,
  output reg [1:0] state_nxt_sel,
  output reg [1:0] a_nxt_sel,
  output reg       b_nxt_sel,
  output reg       div_sgn_nxt_sel,
  output reg       rem_sgn_nxt_sel     //
);
  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------
  assign div_sgn = (a_sign_bit ^ b_sign_bit) & divreq_msg_fn;
  assign rem_sgn = a_sign_bit & divreq_msg_fn;

  // MUX selectors
  always@(*) begin
    // Default
    state_nxt_sel   = 2'd0;
    a_nxt_sel       = 2'd0;
    b_nxt_sel       = 1'b0;
    div_sgn_nxt_sel = 1'b0;
    rem_sgn_nxt_sel = 1'b0;

    if (state == 6'd0) begin      // INIT STAGE: activate after receive req_val
      if (divreq_val) begin
        a_nxt_sel       = 2'd1;
        b_nxt_sel       = 1'b1;
        state_nxt_sel   = 2'd2;
        div_sgn_nxt_sel = 1'b1;
        rem_sgn_nxt_sel = 1'b1;
      end
    end
    else if (state < 6'd32) begin // COMP STAGE: add 32 times
      a_nxt_sel       = 2'd2;
      b_nxt_sel       = 1'b0;
      state_nxt_sel   = 2'd2;
      div_sgn_nxt_sel = 1'b0;
      rem_sgn_nxt_sel = 1'b0;
    end
    else if (state == 6'd32) begin // RESULT STAGE
      a_nxt_sel       = 2'd3;
      b_nxt_sel       = 1'b0;
      state_nxt_sel   = 2'd2;
      div_sgn_nxt_sel = 1'b0;
      rem_sgn_nxt_sel = 1'b0;
    end
    else if (state == 6'd33) begin // OUTPUT STAGE
      a_nxt_sel       = 2'd0;
      b_nxt_sel       = 1'b0;
      state_nxt_sel   = 2'd1;
      div_sgn_nxt_sel = 1'b0;
      rem_sgn_nxt_sel = 1'b0;
    end
  end

endmodule

`endif
