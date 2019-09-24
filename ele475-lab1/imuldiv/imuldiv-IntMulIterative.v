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
  input         mulresp_rdy         // TB ready to accept data
);
  wire   [ 5:0] state;
  wire   [ 1:0] state_nxt_sel, a_nxt_sel, b_nxt_sel, result_nxt_sel;
  wire          sgn_nxt_sel;

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
    .sgn_nxt_sel        (sgn_nxt_sel)
  );

  imuldiv_IntMulIterativeCtrl ctrl
  (
    .state              (state),
    .mulreq_val         (mulreq_val),
    .state_nxt_sel      (state_nxt_sel),
    .a_nxt_sel          (a_nxt_sel),
    .b_nxt_sel          (b_nxt_sel),
    .result_nxt_sel     (result_nxt_sel),
    .sgn_nxt_sel        (sgn_nxt_sel)
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
  input         sgn_nxt_sel
);
  //==== reg/wire declaration ===============================
  reg  [63:0] a_reg;           // Register for storing operand A
  wire [63:0] a_reg_nxt;
  reg  [31:0] b_reg;            // Register for storing operand B
  wire [31:0] b_reg_nxt;
  reg         sgn_reg;          // Register for storing signed or unsigned result
  wire        sgn_reg_nxt;
  reg  [63:0] result_reg;       // Register for storing result
  wire [63:0] result_reg_nxt; 
  reg  [ 5:0] state;            // FSM
  wire [ 5:0] state_nxt;
  reg         mulresp_val;      // Output Reg
  wire        mulresp_val_nxt; 
  reg         mulreq_rdy;       // Output Reg
  wire        mulreq_rdy_nxt;

  wire [31:0] unsigned_a;
  wire [31:0] unsigned_b;
  wire [63:0] a_shift;
  wire [31:0] b_shift;
  wire [63:0] alu_out;
  wire [ 5:0] state_p1;
  wire        is_signed_result;
  wire [63:0] add_mux_out;
  wire [63:0] f_result;         // final_result (sign add_mux_out)

  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // Unsign operands if necessary
  assign unsigned_a = mulreq_msg_a[31] ? (~mulreq_msg_a + 1'b1) : mulreq_msg_a;
  assign unsigned_b = mulreq_msg_b[31] ? (~mulreq_msg_b + 1'b1) : mulreq_msg_b;

  // minor wires
  assign a_shift = a_reg << 1;
  assign b_shift = b_reg >> 1;
  assign alu_out = a_reg + result_reg;
  assign state_p1 = state + 6'd1;
  assign is_signed_result = mulreq_msg_a[31] ^ mulreq_msg_b[31]; 

  // MUX
  assign add_mux_out = b_reg[0] ? alu_out : result_reg;
  assign f_result = sgn_reg ? (~add_mux_out + 64'b1) : add_mux_out;

  // MUX before Registers: Option2 -> Option1 -> Option0
  assign a_reg_nxt = a_nxt_sel[1] ? a_reg :
                     a_nxt_sel[0] ? {32'b0, {unsigned_a}} : a_shift;
  assign b_reg_nxt = b_nxt_sel[1] ? b_reg :
                     b_nxt_sel[0] ? unsigned_b : b_shift;
  assign state_nxt = state_nxt_sel[1] ? state :
                     state_nxt_sel[0] ? 6'b0 : state_p1;
  assign result_reg_nxt = result_nxt_sel[1] ? 
                          (result_nxt_sel[0] ? f_result : result_reg ):
                          (result_nxt_sel[0] ? 64'b0 : add_mux_out);
  assign sgn_reg_nxt = sgn_nxt_sel ? is_signed_result : 1'b0;
  
  //==== OUTPUT SECTION ====
  assign mulresp_msg_result = result_reg;
  assign mulreq_rdy_nxt  = mulresp_rdy && (state == 6'd32);
  assign mulresp_val_nxt = (state == 6'd32);

  //----------------------------------------------------------------------
  // Sequential Logic
  //----------------------------------------------------------------------

  always @( posedge clk or posedge reset ) begin
    if (reset) begin              // Active high reset
      a_reg       <= 64'b0;
      b_reg       <= 32'b0;
      state       <=  6'b0;
      result_reg  <= 64'b0;
      sgn_reg     <=  1'b0;
      mulresp_val <=  1'b0;
      mulreq_rdy  <=  1'b0;
    end
    else if (mulresp_rdy) begin // Stall the pipeline if the response interface is not ready
      a_reg       <= a_reg_nxt;
      b_reg       <= b_reg_nxt;
      state       <= state_nxt;
      result_reg  <= result_reg_nxt;
      sgn_reg     <= sgn_reg_nxt;
      mulresp_val <= mulresp_val_nxt;
      mulreq_rdy  <= mulreq_rdy_nxt;
    end
  end

endmodule

//------------------------------------------------------------------------
// Control Logic
//------------------------------------------------------------------------

module imuldiv_IntMulIterativeCtrl
(
  input [5:0]      state,
  input            mulreq_val,
  output reg [1:0] state_nxt_sel,
  output reg [1:0] a_nxt_sel,
  output reg [1:0] b_nxt_sel,
  output reg [1:0] result_nxt_sel,
  output reg       sgn_nxt_sel   //
);
  //----------------------------------------------------------------------
  // Combinational Logic
  //----------------------------------------------------------------------

  // MUX selectors
  always@(*) begin
    // Default
    state_nxt_sel  = 2'd2;
    a_nxt_sel      = 2'd2;
    b_nxt_sel      = 2'd2;
    result_nxt_sel = 2'd2;
    sgn_nxt_sel    = 1'b0;

    if (state == 6'd0) begin // INIT STAGE: activate after receive req_val
      if (mulreq_val) begin
        a_nxt_sel      = 2'b1;
        b_nxt_sel      = 2'b1;
        state_nxt_sel  = 2'b0;
        result_nxt_sel = 2'b1;
        sgn_nxt_sel    = 1'b1;
      end
    end
    else if (state < 6'd32) begin // COMP STAGE: add 32 times
      a_nxt_sel      = 2'b0;
      b_nxt_sel      = 2'b0;
      state_nxt_sel  = 2'b0;
      result_nxt_sel = 2'b0;
      sgn_nxt_sel    = 1'b1;
    end
    else if (state == 6'd32) begin // COMP STAGE: add 32 times
      a_nxt_sel      = 2'b0;
      b_nxt_sel      = 2'b0;
      state_nxt_sel  = 2'b0;
      result_nxt_sel = 2'd3;
      sgn_nxt_sel    = 1'b1;
    end
    else if (state == 6'd33) begin // COMP STAGE: add 32 times
      a_nxt_sel      = 2'd2;
      b_nxt_sel      = 2'd2;
      state_nxt_sel  = 2'b1;
      result_nxt_sel = 2'd2;
      sgn_nxt_sel    = 1'b0;
    end
  end

endmodule

`endif
