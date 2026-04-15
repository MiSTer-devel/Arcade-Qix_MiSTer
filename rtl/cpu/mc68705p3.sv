//============================================================================
//
// Motorola MC68705P3 Microcontroller Core
// Copyright (C) 2026 Rodimus
//
// Full 6805-HMOS opcode set. Clocked at clk (20 MHz); ce_4m pulses 1/5 clk
// cycles (4 MHz oscillator equivalent). Internally divided by 4 to yield a
// ~1 MHz machine-cycle rate. The micro-FSM advances at clk speed and idles
// in WAIT until enough machine cycles have elapsed to match HMOS timing.
//
// Memory map (11-bit address, 2KB):
//   $000-$00F : I/O & timer registers
//   $010-$07F : 112 bytes internal RAM
//   $080-$7FF : User EPROM (loaded via rom_wr)
//   Vectors  — Timer $7F8/$7F9, /IRQ $7FA/$7FB, SWI $7FC/$7FD, RESET $7FE/$7FF
//
// Ports:
//   PA open-drain (pin = latch | ~ddr), PB/PC push-pull.
//   PC is 4-bit (upper nibble always reads as 0xF).
//
//============================================================================

module mc68705p3 (
    input  wire        clk,
    input  wire        ce_4m,         // 4 MHz osc pulse (1 per 5 clk ticks)
    input  wire        reset,

    input  wire        irq_n,

    // Port A (open-drain, 8-bit)
    input  wire [7:0]  pa_in,
    output wire [7:0]  pa_out,

    // Port B (push-pull, 8-bit)
    input  wire [7:0]  pb_in,
    output wire [7:0]  pb_out,
    output wire [7:0]  pb_ddr,

    // Port C (push-pull, 4-bit)
    input  wire [3:0]  pc_in,
    output wire [3:0]  pc_out,
    output wire [3:0]  pc_ddr,

    // ROM loading
    input  wire        rom_wr,
    input  wire [10:0] rom_addr,
    input  wire [7:0]  rom_data
);

// ---------------------------------------------------------------------------
// Machine-cycle tick — 4 osc cycles per bus cycle
// ---------------------------------------------------------------------------
reg [1:0] osc_div;
reg       mc_tick;

always @(posedge clk) begin
    mc_tick <= 1'b0;
    if (reset) osc_div <= 2'd0;
    else if (ce_4m) begin
        osc_div <= osc_div + 2'd1;
        if (osc_div == 2'd3) mc_tick <= 1'b1;
    end
end

// ---------------------------------------------------------------------------
// EPROM — 2KB, synchronous read (infers M9K)
// ---------------------------------------------------------------------------
(* ramstyle = "M9K" *) reg [7:0] eprom [0:2047];
reg [7:0]  eprom_q;
reg [10:0] mem_raddr;

always @(posedge clk) begin
    if (rom_wr) eprom[rom_addr] <= rom_data;
    eprom_q <= eprom[mem_raddr];
end

// ---------------------------------------------------------------------------
// Internal RAM — 128 entries (asynchronous read, distributed RAM)
// ---------------------------------------------------------------------------
reg [7:0] iram [0:127];

integer i;
initial for (i = 0; i < 128; i = i + 1) iram[i] = 8'h00;

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------
reg [7:0]  A, X;
reg [6:0]  SP;                           // 0x60..0x7F physical
reg [4:0]  CC;                           // {H, I, N, Z, C}
localparam CC_C = 0, CC_Z = 1, CC_N = 2, CC_I = 3, CC_H = 4;
reg [10:0] PC;

reg [7:0]  pa_latch, pa_ddr;
reg [7:0]  pb_latch, pb_ddr_r;
reg [3:0]  pc_latch, pc_ddr_r;
reg [7:0]  tdr, tcr, pcr;
reg        irq_pending, irq_line_r;

// ---------------------------------------------------------------------------
// Port output / read-back
// ---------------------------------------------------------------------------
assign pa_out   = pa_latch | ~pa_ddr;
assign pb_out   = pb_latch;
assign pb_ddr   = pb_ddr_r;
assign pc_out   = pc_latch;
assign pc_ddr   = pc_ddr_r;

wire [7:0] pa_read = (pa_latch & pa_ddr) | (pa_in & ~pa_ddr);
wire [7:0] pb_read = (pb_latch & pb_ddr_r) | (pb_in & ~pb_ddr_r);
wire [7:0] pc_read = {4'hF, (pc_latch & pc_ddr_r) | (pc_in & ~pc_ddr_r)};

// ---------------------------------------------------------------------------
// Memory read — combinational mux (one clk of EPROM latency already buffered
// in eprom_q; the FSM issues mem_raddr one clk before it needs the data).
// ---------------------------------------------------------------------------
reg [10:0] latched_raddr;
wire       mr_is_io    = (latched_raddr[10:4] == 7'd0);
wire       mr_is_ram   = ~mr_is_io & (latched_raddr[10:7] == 4'd0);
wire [7:0] mr_io_val =
    (latched_raddr[3:0] == 4'h0) ? pa_read :
    (latched_raddr[3:0] == 4'h1) ? pb_read :
    (latched_raddr[3:0] == 4'h2) ? pc_read :
    (latched_raddr[3:0] == 4'h4) ? pa_ddr  :
    (latched_raddr[3:0] == 4'h5) ? pb_ddr_r :
    (latched_raddr[3:0] == 4'h6) ? {4'hF, pc_ddr_r} :
    (latched_raddr[3:0] == 4'h8) ? tdr :
    (latched_raddr[3:0] == 4'h9) ? (tcr & 8'hF7) :
    (latched_raddr[3:0] == 4'hB) ? pcr :
                                    8'h00;
wire [7:0] mr_ram_val = iram[latched_raddr[6:0]];
wire [7:0] mem_q      = mr_is_io  ? mr_io_val :
                        mr_is_ram ? mr_ram_val :
                                    eprom_q;

// ---------------------------------------------------------------------------
// Cycle-count table (HMOS)
// ---------------------------------------------------------------------------
function automatic [3:0] opcycles (input [7:0] op);
    begin
        case (op[7:4])
            4'h0: opcycles = 4'd10;
            4'h1: opcycles = 4'd7;
            4'h2: opcycles = 4'd4;
            4'h3: opcycles = 4'd6;
            4'h4: opcycles = 4'd4;
            4'h5: opcycles = 4'd4;
            4'h6: opcycles = 4'd7;
            4'h7: opcycles = 4'd6;
            4'h8: opcycles = (op[3:0]==4'h0) ? 4'd9  :
                             (op[3:0]==4'h1) ? 4'd6  :
                             (op[3:0]==4'h3) ? 4'd11 : 4'd4;
            4'h9: opcycles = 4'd2;
            4'hA: opcycles = (op[3:0]==4'hD) ? 4'd8 : 4'd2;
            4'hB: opcycles = (op[3:0]==4'hD) ? 4'd7 :
                             (op[3:0]==4'h7) ? 4'd5 :
                             (op[3:0]==4'hC) ? 4'd3 :
                             (op[3:0]==4'hF) ? 4'd5 : 4'd4;
            4'hC: opcycles = (op[3:0]==4'hD) ? 4'd8 :
                             (op[3:0]==4'h7) ? 4'd6 :
                             (op[3:0]==4'hC) ? 4'd4 :
                             (op[3:0]==4'hF) ? 4'd6 : 4'd5;
            4'hD: opcycles = (op[3:0]==4'hD) ? 4'd9 :
                             (op[3:0]==4'h7) ? 4'd7 :
                             (op[3:0]==4'hC) ? 4'd5 :
                             (op[3:0]==4'hF) ? 4'd7 : 4'd6;
            4'hE: opcycles = (op[3:0]==4'hD) ? 4'd8 :
                             (op[3:0]==4'h7) ? 4'd6 :
                             (op[3:0]==4'hC) ? 4'd4 :
                             (op[3:0]==4'hF) ? 4'd6 : 4'd5;
            4'hF: opcycles = (op[3:0]==4'hD) ? 4'd7 :
                             (op[3:0]==4'h7) ? 4'd5 :
                             (op[3:0]==4'hC) ? 4'd3 :
                             (op[3:0]==4'hF) ? 4'd5 : 4'd4;
            default: opcycles = 4'd4;
        endcase
    end
endfunction

// ---------------------------------------------------------------------------
// Branch condition evaluator for $2x opcodes
// ---------------------------------------------------------------------------
function automatic branch_true (input [7:0] op, input [4:0] cc);
    reg t;
    begin
        case (op[3:1])
            3'd0: t = 1'b1;
            3'd1: t = ~(cc[CC_C] | cc[CC_Z]);
            3'd2: t = ~cc[CC_C];
            3'd3: t = ~cc[CC_Z];
            3'd4: t = ~cc[CC_H];
            3'd5: t = ~cc[CC_N];
            3'd6: t = ~cc[CC_I];
            3'd7: t = 1'b1;    // BIL: IRQ line — treat as always true (no /IRQ pin test)
            default: t = 1'b0;
        endcase
        branch_true = t ^ op[0];
    end
endfunction

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
localparam [5:0]
    S_BOOT0  = 6'd0,    // start fetching reset vector
    S_VEC1   = 6'd1,    // latch vec hi
    S_VEC2   = 6'd2,    // fetch vec lo
    S_VEC3   = 6'd3,    // latch vec lo → PC
    S_FETCH  = 6'd4,    // issue opcode read
    S_FETCHd = 6'd5,    // latch opcode, decode
    S_OP1    = 6'd6,    // issue operand1 read
    S_OP1d   = 6'd7,    // latch operand1
    S_OP2    = 6'd8,    // issue operand2 read
    S_OP2d   = 6'd9,    // latch operand2
    S_MR     = 6'd10,   // issue memread at EA
    S_MRd    = 6'd11,   // latch mem data
    S_EXEC   = 6'd12,   // execute combinational
    S_WB     = 6'd13,   // write-back for RMW / STA / STX
    S_PUSH   = 6'd14,   // sequenced push (SWI / JSR / BSR / IRQ)
    S_PULL   = 6'd15,   // sequenced pull (RTS / RTI)
    S_WAIT   = 6'd16;

reg [5:0]  state;
reg [7:0]  mem_val;
reg [10:0] ea;
reg [3:0]  cyc_left;
reg [3:0]  seq;              // push/pull sub-step
reg        wr_req;           // pulse: perform write next clk
reg [10:0] wr_addr;
reg [7:0]  wr_data;
reg        irq_service;      // servicing an interrupt (not SWI) — affects vector
reg        swi_service;      // servicing SWI

// ---------------------------------------------------------------------------
// Deferred write — one clk after wr_req is set
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    if (reset) begin
        pa_latch <= 8'hFF;
        pa_ddr   <= 8'h00;
        pb_latch <= 8'h00;
        pb_ddr_r <= 8'h00;
        pc_latch <= 4'h0;
        pc_ddr_r <= 4'h0;
        tdr      <= 8'hFF;
        tcr      <= 8'h7F;
        pcr      <= 8'hFF;
    end else if (wr_req) begin
        if (wr_addr[10:4] == 7'd0) begin
            case (wr_addr[3:0])
                4'h0: pa_latch <= wr_data;
                4'h1: pb_latch <= wr_data;
                4'h2: pc_latch <= wr_data[3:0];
                4'h4: pa_ddr   <= wr_data;
                4'h5: pb_ddr_r <= wr_data;
                4'h6: pc_ddr_r <= wr_data[3:0];
                4'h8: tdr      <= wr_data;
                4'h9: tcr      <= (tcr & {wr_data[7], 7'd0}) | (wr_data & 8'h77);
                4'hB: pcr      <= wr_data;
                default: ;
            endcase
        end else if (wr_addr[10:7] == 4'd0) begin
            iram[wr_addr[6:0]] <= wr_data;
        end
    end
end

// ---------------------------------------------------------------------------
// IRQ edge latch
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    if (reset) begin
        irq_pending <= 1'b0;
        irq_line_r  <= 1'b1;
    end else begin
        irq_line_r <= irq_n;
        if (~irq_n & irq_line_r) irq_pending <= 1'b1;
        // also hold while low (level-triggered behavior)
        if (~irq_n) irq_pending <= 1'b1;
        if (irq_service && state == S_PUSH && seq == 4'd5) irq_pending <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// Combinational helpers
// ---------------------------------------------------------------------------
wire [7:0] rel_extend  = operand1;
wire signed [7:0] sr_signed = operand1;
wire signed [7:0] sr2_signed = operand2;

// H flag from a+b → r (half-carry: carry out of bit 3)
function automatic hc (input [7:0] a, input [7:0] b, input [7:0] r);
    hc = (a[3] & b[3]) | (b[3] & ~r[3]) | (~r[3] & a[3]);
endfunction

// Set {N,Z} preserving {H,I,C}
function automatic [4:0] fz (input [4:0] ci, input [7:0] v);
    fz = {ci[4], ci[3], v[7], (v == 8'h00), ci[0]};
endfunction

// Set {N,Z,C} preserving {H,I}
function automatic [4:0] fzc (input [4:0] ci, input [7:0] v, input c);
    fzc = {ci[4], ci[3], v[7], (v == 8'h00), c};
endfunction

// Set {H,N,Z,C} preserving I
function automatic [4:0] fhzc (input [4:0] ci, input h, input [7:0] v, input c);
    fhzc = {h, ci[3], v[7], (v == 8'h00), c};
endfunction

// ---------------------------------------------------------------------------
// Helper: picks between IMM operand byte (for $Ax) and memory read (others)
// ---------------------------------------------------------------------------
reg [7:0] opcode;
reg [7:0] operand1, operand2;

function automatic [7:0] mem_q_or_op;
    begin
        if (opcode[7:4] == 4'hA) mem_q_or_op = operand1;
        else                     mem_q_or_op = mem_q;
    end
endfunction

// ---------------------------------------------------------------------------
// Main state machine
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    if (reset) begin
        state         <= S_BOOT0;
        A             <= 8'h00;
        X             <= 8'h00;
        SP            <= 7'h7F;
        CC            <= 5'b01000;  // I set, rest clear
        PC            <= 11'h000;
        cyc_left      <= 4'd0;
        seq           <= 4'd0;
        wr_req        <= 1'b0;
        mem_raddr     <= 11'h7FE;
        latched_raddr <= 11'h7FE;
        opcode        <= 8'h00;
        operand1      <= 8'h00;
        operand2      <= 8'h00;
        mem_val       <= 8'h00;
        ea            <= 11'h000;
        irq_service   <= 1'b0;
        swi_service   <= 1'b0;
    end else begin
        // Defaults (deasserted every clock)
        wr_req <= 1'b0;

        case (state)
            // ---------- Reset vector load ----------
            S_BOOT0: begin
                mem_raddr     <= 11'h7FE;
                latched_raddr <= 11'h7FE;
                state         <= S_VEC1;
            end
            S_VEC1: begin
                // eprom_q is pipelined; wait one clk
                mem_raddr     <= 11'h7FF;
                latched_raddr <= 11'h7FF;
                state         <= S_VEC2;
            end
            S_VEC2: begin
                // eprom_q now holds vec high (from 7FE)
                PC[10:8] <= eprom_q[2:0];
                // fall through: wait for 7FF fetch result
                state    <= S_VEC3;
            end
            S_VEC3: begin
                PC[7:0]  <= eprom_q;
                state    <= S_FETCH;
                cyc_left <= 4'd0;
            end

            // ---------- Main fetch cycle ----------
            S_FETCH: begin
                // Check for pending interrupt (level-sensitive external /IRQ)
                if (~irq_n & ~CC[CC_I]) begin
                    // Trigger interrupt service: push PCL, PCH, X, A, CC
                    irq_service <= 1'b1;
                    seq         <= 4'd0;
                    cyc_left    <= 4'd11;
                    state       <= S_PUSH;
                end else begin
                    mem_raddr     <= PC;
                    latched_raddr <= PC;
                    PC            <= PC + 11'd1;
                    state         <= S_FETCHd;
                end
            end
            S_FETCHd: begin
                // one clk for eprom_q to update
                state <= S_OP1;
            end

            S_OP1: begin
                opcode   <= mem_q;
                cyc_left <= opcycles(mem_q);
                // Dispatch per opcode class to decide next steps
                casez (mem_q)
                    // Inherent on A/X, no operand
                    8'b0100_????, 8'b0101_????: begin
                        state <= S_EXEC;
                    end
                    // $8x: RTI ($80), RTS ($81), SWI ($83)
                    8'h80: begin seq <= 4'd0; state <= S_PULL; end          // RTI
                    8'h81: begin seq <= 4'd10; state <= S_PULL; end         // RTS
                    8'h83: begin
                        swi_service <= 1'b1;
                        seq         <= 4'd0;
                        state       <= S_PUSH;
                    end
                    // $9x inherent
                    8'h97, 8'h98, 8'h99, 8'h9A, 8'h9B, 8'h9C, 8'h9D, 8'h9F: begin
                        state <= S_EXEC;
                    end
                    // $7x RMW indexed (EA = X): issue mem read
                    8'b0111_????: begin
                        ea            <= {3'd0, X};
                        mem_raddr     <= {3'd0, X};
                        latched_raddr <= {3'd0, X};
                        state         <= S_MRd;
                    end
                    // $Fx ops indexed, no offset (EA = X)
                    8'b1111_????: begin
                        ea            <= {3'd0, X};
                        mem_raddr     <= {3'd0, X};
                        latched_raddr <= {3'd0, X};
                        state         <= S_MRd;
                    end
                    // All others need at least one operand byte
                    default: begin
                        mem_raddr     <= PC;
                        latched_raddr <= PC;
                        PC            <= PC + 11'd1;
                        state         <= S_OP1d;
                    end
                endcase
            end

            S_OP1d: state <= S_OP2;

            S_OP2: begin
                operand1 <= mem_q;
                casez (opcode)
                    // $0x BRSET/BRCLR: fetch rel byte next
                    8'b0000_????: begin
                        mem_raddr     <= PC;
                        latched_raddr <= PC;
                        PC            <= PC + 11'd1;
                        state         <= S_OP2d;
                    end
                    // $1x BSET/BCLR: read direct byte, modify, writeback
                    8'b0001_????: begin
                        ea            <= {3'd0, mem_q};
                        mem_raddr     <= {3'd0, mem_q};
                        latched_raddr <= {3'd0, mem_q};
                        state         <= S_MRd;
                    end
                    // $2x branch: operand1 is rel
                    8'b0010_????: state <= S_EXEC;
                    // $3x RMW direct: EA = operand1
                    8'b0011_????: begin
                        ea            <= {3'd0, mem_q};
                        mem_raddr     <= {3'd0, mem_q};
                        latched_raddr <= {3'd0, mem_q};
                        state         <= S_MRd;
                    end
                    // $6x RMW indexed + 1 byte: EA = operand1 + X
                    8'b0110_????: begin
                        ea            <= {3'd0, mem_q + X};
                        mem_raddr     <= {3'd0, mem_q + X};
                        latched_raddr <= {3'd0, mem_q + X};
                        state         <= S_MRd;
                    end
                    // $Ax immediate: data byte = operand1 (no memread)
                    8'b1010_????: begin
                        mem_val <= mem_q;
                        state   <= S_EXEC;
                    end
                    // $Bx direct
                    8'b1011_????: begin
                        ea            <= {3'd0, mem_q};
                        mem_raddr     <= {3'd0, mem_q};
                        latched_raddr <= {3'd0, mem_q};
                        state         <= S_MRd;
                    end
                    // $Cx extended / $Dx IX2: need second byte
                    8'b1100_????, 8'b1101_????: begin
                        mem_raddr     <= PC;
                        latched_raddr <= PC;
                        PC            <= PC + 11'd1;
                        state         <= S_OP2d;
                    end
                    // $Ex IX1
                    8'b1110_????: begin
                        ea            <= {3'd0, mem_q + X};
                        mem_raddr     <= {3'd0, mem_q + X};
                        latched_raddr <= {3'd0, mem_q + X};
                        state         <= S_MRd;
                    end
                    default: state <= S_EXEC;
                endcase
            end

            S_OP2d: state <= S_MR;

            S_MR: begin
                operand2 <= mem_q;
                casez (opcode)
                    8'b0000_????: begin
                        // BRSET/BRCLR: operand1 = direct addr, operand2 = rel.
                        // Now read the direct byte.
                        ea            <= {3'd0, operand1};
                        mem_raddr     <= {3'd0, operand1};
                        latched_raddr <= {3'd0, operand1};
                        state         <= S_MRd;
                    end
                    8'b1100_????: begin
                        // EXT: EA = {operand1[2:0], mem_q}
                        ea            <= {operand1[2:0], mem_q};
                        mem_raddr     <= {operand1[2:0], mem_q};
                        latched_raddr <= {operand1[2:0], mem_q};
                        state         <= S_MRd;
                    end
                    8'b1101_????: begin
                        // IX2: EA = {operand1, mem_q} + X (11-bit)
                        ea            <= ({operand1, mem_q} + {3'd0, X}) & 11'h7FF;
                        mem_raddr     <= ({operand1, mem_q} + {3'd0, X}) & 11'h7FF;
                        latched_raddr <= ({operand1, mem_q} + {3'd0, X}) & 11'h7FF;
                        state         <= S_MRd;
                    end
                    default: state <= S_EXEC;
                endcase
            end

            S_MRd: state <= S_EXEC;

            // ----------------- EXECUTE -----------------
            S_EXEC: begin
                // Default next state: waiting for cycle count
                state <= S_WAIT;
                mem_val <= mem_q;
                casez (opcode)
                    // BRSET / BRCLR
                    8'b0000_????: begin
                        CC[CC_C] <= mem_q[opcode[3:1]];
                        if ((~opcode[0] &  mem_q[opcode[3:1]]) |
                            ( opcode[0] & ~mem_q[opcode[3:1]])) begin
                            PC <= PC + {{3{operand2[7]}}, operand2};
                        end
                    end
                    // BSET / BCLR
                    8'b0001_????: begin
                        wr_req   <= 1'b1;
                        wr_addr  <= ea;
                        wr_data  <= opcode[0] ? (mem_q & ~(8'h01 << opcode[3:1]))
                                              : (mem_q |  (8'h01 << opcode[3:1]));
                    end
                    // Branches $2x
                    8'b0010_????: begin
                        if (branch_true(opcode, CC))
                            PC <= PC + {{3{operand1[7]}}, operand1};
                    end
                    // RMW — memory ($3x direct, $6x IX1, $7x IX)
                    8'b0011_????, 8'b0110_????, 8'b0111_????: begin
                        // writeback by default except TST/BIT
                        case (opcode[3:0])
                            4'h0: begin
                                wr_req <= 1'b1; wr_addr <= ea;
                                wr_data <= (~mem_q) + 8'd1;      // NEG
                                CC <= fzc(CC, (~mem_q) + 8'd1, (~mem_q) + 8'd1 != 0);
                            end
                            4'h3: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= ~mem_q;
                                CC <= {CC[4:3], ~mem_q[7],
                                       (~mem_q) == 8'h00, 1'b1};
                            end
                            4'h4: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= {1'b0, mem_q[7:1]};
                                CC <= {CC[4:3], 1'b0, mem_q[7:1] == 7'd0, mem_q[0]};
                            end
                            4'h6: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= {CC[CC_C], mem_q[7:1]};
                                CC <= {CC[4:3], CC[CC_C], {CC[CC_C], mem_q[7:1]} == 8'h00, mem_q[0]};
                            end
                            4'h7: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= {mem_q[7], mem_q[7:1]};
                                CC <= {CC[4:3], mem_q[7], {mem_q[7], mem_q[7:1]} == 8'h00, mem_q[0]};
                            end
                            4'h8: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= {mem_q[6:0], 1'b0};
                                CC <= {CC[4:3], mem_q[6], mem_q[6:0] == 7'd0, mem_q[7]};
                            end
                            4'h9: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= {mem_q[6:0], CC[CC_C]};
                                CC <= {CC[4:3], mem_q[6], {mem_q[6:0], CC[CC_C]} == 8'h00, mem_q[7]};
                            end
                            4'hA: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= mem_q - 8'd1;
                                CC <= fz(CC, mem_q - 8'd1);
                            end
                            4'hC: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= mem_q + 8'd1;
                                CC <= fz(CC, mem_q + 8'd1);
                            end
                            4'hD: CC <= fz(CC, mem_q);    // TST (no wb)
                            4'hF: begin
                                wr_req <= 1'b1; wr_addr <= ea; wr_data <= 8'h00;
                                CC <= {CC[4:3], 1'b0, 1'b1, CC[CC_C]};
                            end
                            default: ;
                        endcase
                    end
                    // RMW on A ($4x)
                    8'b0100_????: begin
                        case (opcode[3:0])
                            4'h0: begin A <= (~A) + 8'd1;
                                        CC <= fzc(CC, (~A) + 8'd1, ((~A) + 8'd1) != 0); end
                            4'h3: begin A <= ~A;
                                        CC <= {CC[4:3], ~A[7], (~A) == 8'h00, 1'b1}; end
                            4'h4: begin A <= {1'b0, A[7:1]};
                                        CC <= {CC[4:3], 1'b0, A[7:1] == 7'd0, A[0]}; end
                            4'h6: begin A <= {CC[CC_C], A[7:1]};
                                        CC <= {CC[4:3], CC[CC_C], {CC[CC_C], A[7:1]} == 8'h00, A[0]}; end
                            4'h7: begin A <= {A[7], A[7:1]};
                                        CC <= {CC[4:3], A[7], {A[7], A[7:1]} == 8'h00, A[0]}; end
                            4'h8: begin A <= {A[6:0], 1'b0};
                                        CC <= {CC[4:3], A[6], A[6:0] == 7'd0, A[7]}; end
                            4'h9: begin A <= {A[6:0], CC[CC_C]};
                                        CC <= {CC[4:3], A[6], {A[6:0], CC[CC_C]} == 8'h00, A[7]}; end
                            4'hA: begin A <= A - 8'd1; CC <= fz(CC, A - 8'd1); end
                            4'hC: begin A <= A + 8'd1; CC <= fz(CC, A + 8'd1); end
                            4'hD: CC <= fz(CC, A);
                            4'hF: begin A <= 8'h00; CC <= {CC[4:3], 1'b0, 1'b1, CC[CC_C]}; end
                            default: ;
                        endcase
                    end
                    // RMW on X ($5x)
                    8'b0101_????: begin
                        case (opcode[3:0])
                            4'h0: begin X <= (~X) + 8'd1;
                                        CC <= fzc(CC, (~X) + 8'd1, ((~X) + 8'd1) != 0); end
                            4'h3: begin X <= ~X;
                                        CC <= {CC[4:3], ~X[7], (~X) == 8'h00, 1'b1}; end
                            4'h4: begin X <= {1'b0, X[7:1]};
                                        CC <= {CC[4:3], 1'b0, X[7:1] == 7'd0, X[0]}; end
                            4'h6: begin X <= {CC[CC_C], X[7:1]};
                                        CC <= {CC[4:3], CC[CC_C], {CC[CC_C], X[7:1]} == 8'h00, X[0]}; end
                            4'h7: begin X <= {X[7], X[7:1]};
                                        CC <= {CC[4:3], X[7], {X[7], X[7:1]} == 8'h00, X[0]}; end
                            4'h8: begin X <= {X[6:0], 1'b0};
                                        CC <= {CC[4:3], X[6], X[6:0] == 7'd0, X[7]}; end
                            4'h9: begin X <= {X[6:0], CC[CC_C]};
                                        CC <= {CC[4:3], X[6], {X[6:0], CC[CC_C]} == 8'h00, X[7]}; end
                            4'hA: begin X <= X - 8'd1; CC <= fz(CC, X - 8'd1); end
                            4'hC: begin X <= X + 8'd1; CC <= fz(CC, X + 8'd1); end
                            4'hD: CC <= fz(CC, X);
                            4'hF: begin X <= 8'h00; CC <= {CC[4:3], 1'b0, 1'b1, CC[CC_C]}; end
                            default: ;
                        endcase
                    end
                    // $9x inherent (TAX/CLC/SEC/CLI/SEI/RSP/NOP/TXA)
                    8'h97: X          <= A;
                    8'h98: CC[CC_C]   <= 1'b0;
                    8'h99: CC[CC_C]   <= 1'b1;
                    8'h9A: CC[CC_I]   <= 1'b0;
                    8'h9B: CC[CC_I]   <= 1'b1;
                    8'h9C: SP         <= 7'h7F;
                    8'h9D: ;
                    8'h9F: A          <= X;
                    // $Ax (IMM), $Bx (DIR), $Cx (EXT), $Dx (IX2), $Ex (IX1), $Fx (IX)
                    // mem_val holds the data byte (either operand1 for IMM, or mem_q for memread)
                    8'b1???_????: begin
                        case (opcode[3:0])
                            4'h0: begin  // SUBA
                                A  <= A - mem_q_or_op();
                                CC <= fzc(CC, A - mem_q_or_op(),
                                          ({1'b0,A} < {1'b0,mem_q_or_op()}));
                            end
                            4'h1: begin  // CMPA
                                CC <= fzc(CC, A - mem_q_or_op(),
                                          ({1'b0,A} < {1'b0,mem_q_or_op()}));
                            end
                            4'h2: begin  // SBCA
                                A  <= A - mem_q_or_op() - {7'd0,CC[CC_C]};
                                CC <= fzc(CC, A - mem_q_or_op() - {7'd0,CC[CC_C]},
                                          ({1'b0,A} < {1'b0,mem_q_or_op()} + {8'd0,CC[CC_C]}));
                            end
                            4'h3: begin  // CPX
                                CC <= fzc(CC, X - mem_q_or_op(),
                                          ({1'b0,X} < {1'b0,mem_q_or_op()}));
                            end
                            4'h4: begin  // ANDA
                                A  <= A & mem_q_or_op();
                                CC <= fz(CC, A & mem_q_or_op());
                            end
                            4'h5: begin  // BITA
                                CC <= fz(CC, A & mem_q_or_op());
                            end
                            4'h6: begin  // LDA
                                A  <= mem_q_or_op();
                                CC <= fz(CC, mem_q_or_op());
                            end
                            4'h7: begin  // STA (DIR/EXT/IX2/IX1/IX)
                                if (opcode[7:4] != 4'hA) begin
                                    wr_req  <= 1'b1;
                                    wr_addr <= ea;
                                    wr_data <= A;
                                    CC      <= fz(CC, A);
                                end
                            end
                            4'h8: begin  // EORA
                                A  <= A ^ mem_q_or_op();
                                CC <= fz(CC, A ^ mem_q_or_op());
                            end
                            4'h9: begin  // ADCA
                                A  <= A + mem_q_or_op() + {7'd0,CC[CC_C]};
                                CC <= fhzc(CC, hc(A, mem_q_or_op(), A + mem_q_or_op() + {7'd0,CC[CC_C]}),
                                           A + mem_q_or_op() + {7'd0,CC[CC_C]},
                                           ({1'b0,A} + {1'b0,mem_q_or_op()} + {8'd0,CC[CC_C]}) > 9'h0FF);
                            end
                            4'hA: begin  // ORA
                                A  <= A | mem_q_or_op();
                                CC <= fz(CC, A | mem_q_or_op());
                            end
                            4'hB: begin  // ADDA
                                A  <= A + mem_q_or_op();
                                CC <= fhzc(CC, hc(A, mem_q_or_op(), A + mem_q_or_op()),
                                           A + mem_q_or_op(),
                                           ({1'b0,A} + {1'b0,mem_q_or_op()}) > 9'h0FF);
                            end
                            4'hC: begin  // JMP
                                if (opcode[7:4] != 4'hA) PC <= ea;
                            end
                            4'hD: begin
                                // BSR ($AD) / JSR (rest): push PC then jump
                                seq   <= 4'd10;     // JSR/BSR marker
                                if (opcode == 8'hAD)
                                    ea <= PC + {{3{operand1[7]}}, operand1};
                                // else ea already set
                                state <= S_PUSH;
                            end
                            4'hE: begin  // LDX
                                X  <= mem_q_or_op();
                                CC <= fz(CC, mem_q_or_op());
                            end
                            4'hF: begin  // STX
                                if (opcode[7:4] != 4'hA) begin
                                    wr_req  <= 1'b1;
                                    wr_addr <= ea;
                                    wr_data <= X;
                                    CC      <= fz(CC, X);
                                end
                            end
                            default: ;
                        endcase
                    end
                    default: ;
                endcase
            end

            // -------- Push sequencer --------
            // seq==0..4  : IRQ or SWI (push PCL/PCH/X/A/CC)
            // seq==10    : JSR/BSR first push (PCL)
            // seq==11    : JSR/BSR second push (PCH), then jump
            S_PUSH: begin
                case (seq)
                    4'd0:  begin wr_req <= 1'b1; wr_addr <= {4'd0, SP}; wr_data <= PC[7:0];
                                 SP <= SP - 7'd1; seq <= 4'd1; end
                    4'd1:  begin wr_req <= 1'b1; wr_addr <= {4'd0, SP}; wr_data <= {5'd0, PC[10:8]};
                                 SP <= SP - 7'd1; seq <= 4'd2; end
                    4'd2:  begin wr_req <= 1'b1; wr_addr <= {4'd0, SP}; wr_data <= X;
                                 SP <= SP - 7'd1; seq <= 4'd3; end
                    4'd3:  begin wr_req <= 1'b1; wr_addr <= {4'd0, SP}; wr_data <= A;
                                 SP <= SP - 7'd1; seq <= 4'd4; end
                    4'd4:  begin wr_req <= 1'b1; wr_addr <= {4'd0, SP}; wr_data <= {3'd0, CC};
                                 SP <= SP - 7'd1; seq <= 4'd5; end
                    4'd5:  begin
                        // Load vector: /IRQ=$7FA, Timer=$7F8, SWI=$7FC
                        CC[CC_I] <= 1'b1;
                        if (swi_service) begin
                            mem_raddr     <= 11'h7FC;
                            latched_raddr <= 11'h7FC;
                        end else begin
                            mem_raddr     <= 11'h7FA;
                            latched_raddr <= 11'h7FA;
                        end
                        seq   <= 4'd6;
                    end
                    4'd6:  begin
                        // wait 1 clk for eprom_q
                        mem_raddr     <= (swi_service ? 11'h7FD : 11'h7FB);
                        latched_raddr <= (swi_service ? 11'h7FD : 11'h7FB);
                        seq <= 4'd7;
                    end
                    4'd7:  begin
                        PC[10:8] <= eprom_q[2:0];
                        seq <= 4'd8;
                    end
                    4'd8:  begin
                        PC[7:0] <= eprom_q;
                        state  <= S_WAIT;
                        irq_service <= 1'b0;
                        swi_service <= 1'b0;
                    end
                    // JSR/BSR path
                    4'd10: begin wr_req <= 1'b1; wr_addr <= {4'd0, SP}; wr_data <= PC[7:0];
                                 SP <= SP - 7'd1; seq <= 4'd11; end
                    4'd11: begin wr_req <= 1'b1; wr_addr <= {4'd0, SP}; wr_data <= {5'd0, PC[10:8]};
                                 SP <= SP - 7'd1; seq <= 4'd12; end
                    4'd12: begin PC <= ea; state <= S_WAIT; end
                    default: state <= S_WAIT;
                endcase
            end

            // -------- Pull sequencer (RTI / RTS) --------
            // RTI: seq 0..4 pulls CC, A, X, PCH, PCL
            // RTS: seq 10..11 pulls PCH, PCL
            S_PULL: begin
                case (seq)
                    4'd0: begin SP <= SP + 7'd1; mem_raddr <= {4'd0, SP + 7'd1};
                                latched_raddr <= {4'd0, SP + 7'd1}; seq <= 4'd1; end
                    4'd1: begin seq <= 4'd2; end
                    4'd2: begin CC <= mem_q[4:0];
                                SP <= SP + 7'd1; mem_raddr <= {4'd0, SP + 7'd1};
                                latched_raddr <= {4'd0, SP + 7'd1}; seq <= 4'd3; end
                    4'd3: begin A <= mem_q;
                                SP <= SP + 7'd1; mem_raddr <= {4'd0, SP + 7'd1};
                                latched_raddr <= {4'd0, SP + 7'd1}; seq <= 4'd4; end
                    4'd4: begin X <= mem_q;
                                SP <= SP + 7'd1; mem_raddr <= {4'd0, SP + 7'd1};
                                latched_raddr <= {4'd0, SP + 7'd1}; seq <= 4'd5; end
                    4'd5: begin PC[10:8] <= mem_q[2:0];
                                SP <= SP + 7'd1; mem_raddr <= {4'd0, SP + 7'd1};
                                latched_raddr <= {4'd0, SP + 7'd1}; seq <= 4'd6; end
                    4'd6: begin PC[7:0] <= mem_q; state <= S_WAIT; end
                    // RTS
                    4'd10: begin SP <= SP + 7'd1; mem_raddr <= {4'd0, SP + 7'd1};
                                 latched_raddr <= {4'd0, SP + 7'd1}; seq <= 4'd11; end
                    4'd11: begin seq <= 4'd12; end
                    4'd12: begin PC[10:8] <= mem_q[2:0];
                                 SP <= SP + 7'd1; mem_raddr <= {4'd0, SP + 7'd1};
                                 latched_raddr <= {4'd0, SP + 7'd1}; seq <= 4'd13; end
                    4'd13: begin PC[7:0] <= mem_q; state <= S_WAIT; end
                    default: state <= S_WAIT;
                endcase
            end

            // -------- Cycle-accurate wait --------
            S_WAIT: begin
                if (mc_tick) begin
                    if (cyc_left == 4'd0) begin
                        state <= S_FETCH;
                    end else begin
                        cyc_left <= cyc_left - 4'd1;
                    end
                end
            end

            default: state <= S_WAIT;
        endcase
    end
end

endmodule
