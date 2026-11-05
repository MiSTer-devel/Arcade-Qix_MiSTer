//============================================================================
//  tms5220.sv -- TMS5220 speech synthesizer, Speak External (FIFO) path.
//  Ported from MAME src/devices/sound/tms5220.cpp (default build, analog output via clip_analog,
//  instant data_w / status_r). Tables: tms5220_tables.vh, generated from MAME tms5110r.hxx.
//  Bit-exact against MAME: verilator/tms5220_cosim/.
//  One ce_sample = one sample (TMS clock / 80): bit-serial frame parse, one shared multiplier for the
//  20 lattice products; CPU accesses wait for the sequencer so they land between samples.
//  No VSM: Speak / Read Byte read all ones (as MAME with no VSM attached).
//============================================================================

module tms5220
(
    input  wire        clk,
    input  wire        reset,
    input  wire        ce_sample,     // one pulse per output sample (TMS clock / 80)

    input  wire        wr,            // CPU write strobe (one clk): MAME data_w()
    input  wire  [7:0] din,
    input  wire        rd,            // CPU read strobe (one clk): MAME status_r() side effect (clears INT)
    output reg   [7:0] dout,          // status: TS BL BE 00000 (or the read-byte register after a Read Byte)
    output wire        int_n,         // /INT
    output wire        ready_n,       // /READY (MAME ready_read(), instant mode)

    output reg  signed [15:0] audio,  // MAME process() sample (clip_analog: analog SPK pin, 16-bit range)
    output reg         audio_valid
);

`include "tms5220_tables.vh"

    localparam [7:0] NOCOMMAND = 8'hFF;

    // ---------------- chip state (names follow MAME's m_*) ----------------
    reg  [7:0] fifo [0:15];
    reg  [3:0] fifo_head, fifo_tail;
    reg  [7:0] fifo_count;            // uint8_t in MAME (wraps)
    reg  [2:0] fifo_bits_taken;
    reg        SPEN, DDIS, TALK, TALKD, prev_talk, buffer_low, buffer_empty, irq_pin;
    reg  [7:0] command_register;
    reg        RDB_flag;
    reg  [7:0] read_byte_register;
    reg        OLDE, OLDP;
    reg  [3:0] new_e_idx;
    reg  [5:0] new_p_idx;
    reg  [4:0] new_k_idx [0:9];
    reg signed [15:0] cur_e, cur_p;   // int16_t
    reg signed [15:0] cur_k [0:9];
    reg [15:0] prev_e;                // uint16_t
    reg  [1:0] subcycle;
    reg  [3:0] PC;
    reg  [2:0] IP;
    reg        inhibit, uv_zpar, zpar, pitch_zero;
    reg  [8:0] pitch_count;           // &0x1FF in MAME
    reg [15:0] RNG;
    reg signed [15:0] exc;
    reg signed [23:0] u [0:10];       // int32_t in MAME; |values| < 2^20 by construction
    reg signed [23:0] x [0:9];
    reg        io_ready, data_latched;

    wire talk_status = SPEN | TALKD;
    assign int_n   = ~irq_pin;
    assign ready_n = ~(((fifo_count < 8'd16) | ~DDIS) & io_ready);

    // ---------------- MAME update_fifo_status_and_ints() ----------------
    task automatic update_fifo;
        begin
            if (fifo_count <= 8'd8) begin
                if (!buffer_low) begin buffer_low = 1'b1; irq_pin = 1'b1; end
            end else
                buffer_low = 1'b0;
            if (fifo_count == 8'd0) begin
                if (!buffer_empty) begin buffer_empty = 1'b1; irq_pin = 1'b1; end
                if (DDIS) begin TALK = 1'b0; SPEN = 1'b0; end
            end else
                buffer_empty = 1'b0;
            if (prev_talk && !(SPEN | TALKD)) begin
                irq_pin = 1'b1;
                DDIS = 1'b0;
                prev_talk = 1'b0;
            end
            prev_talk = SPEN | TALKD;
        end
    endtask

    // ---------------- MAME read_bits(1) step, Speak External path ----------------
    reg rb_bit;
    task automatic read_bit;
        begin
            if (DDIS) begin
                rb_bit = fifo[fifo_head][fifo_bits_taken];
                if (fifo_bits_taken == 3'd7) begin
                    fifo_count = fifo_count - 8'd1;
                    fifo[fifo_head] = 8'd0;
                    fifo_head = fifo_head + 4'd1;
                    fifo_bits_taken = 3'd0;
                    update_fifo;
                end else
                    fifo_bits_taken = fifo_bits_taken + 3'd1;
            end else
                rb_bit = 1'b1;          // no VSM attached: input floats high
        end
    endtask

    task automatic init_new_frame;      // "semi-hack but matches idle state observed on chip"
        integer i;
        begin
            new_e_idx = 4'd0;
            new_p_idx = 6'd0;
            for (i = 0; i < 4; i = i + 1)  new_k_idx[i] = 5'h00;
            for (i = 4; i < 7; i = i + 1)  new_k_idx[i] = 5'h0F;
            for (i = 7; i < 10; i = i + 1) new_k_idx[i] = 5'h07;
        end
    endtask

    task automatic chip_reset;          // MAME device_reset()
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1) fifo[i] = 8'd0;
            fifo_head = 4'd0; fifo_tail = 4'd0; fifo_count = 8'd0; fifo_bits_taken = 3'd0;
            SPEN = 1'b0; DDIS = 1'b0; TALK = 1'b0; TALKD = 1'b0; prev_talk = 1'b0; irq_pin = 1'b0;
            buffer_empty = 1'b1; buffer_low = 1'b1;
            command_register = NOCOMMAND; data_latched = 1'b0; RDB_flag = 1'b0;
            new_e_idx = 4'd0; cur_e = 16'sd0; prev_e = 16'd0; new_p_idx = 6'd0; cur_p = 16'sd0;
            zpar = 1'b0; uv_zpar = 1'b0;
            for (i = 0; i < 10; i = i + 1) begin new_k_idx[i] = 5'd0; cur_k[i] = 16'sd0; end
            inhibit = 1'b1; subcycle = 2'd0; pitch_count = 9'd0; PC = 4'd0;
            OLDE = 1'b1; OLDP = 1'b1; IP = 3'd0; RNG = 16'h1FFF;
            for (i = 0; i < 11; i = i + 1) u[i] = 24'sd0;
            for (i = 0; i < 10; i = i + 1) x[i] = 24'sd0;
        end
    endtask

    // ---------------- MAME data_write() / process_command() ----------------
    task automatic data_write(input [7:0] d);
        reg old_bl;
        begin
            old_bl = buffer_low;
            if (DDIS) begin
                if (fifo_count < 8'd16) begin
                    fifo[fifo_tail] = d;
                    fifo_tail = fifo_tail + 4'd1;
                    fifo_count = fifo_count + 8'd1;
                    update_fifo;
                    if (!SPEN && old_bl && !buffer_low) begin
                        zpar = 1'b1; uv_zpar = 1'b1; OLDE = 1'b1; OLDP = 1'b1;
                        SPEN = 1'b1;
                        TALK = 1'b1;                    // FAST_START_HACK
                        init_new_frame;
                    end
                end
                data_latched = 1'b0;
            end else begin
                command_register = d;
                case (d[6:4])
                    3'h1: begin                         // read byte (no VSM: all ones)
                        if (!(SPEN | TALKD)) begin
                            read_byte_register = 8'hFF;
                            RDB_flag = 1'b1;
                            command_register = NOCOMMAND;
                        end
                    end
                    3'h0, 3'h2: command_register = NOCOMMAND;   // NOP / set rate (5220: NOP)
                    3'h3: if (!(SPEN | TALKD)) begin RDB_flag = 1'b0; command_register = NOCOMMAND; end
                    3'h4: if (!(SPEN | TALKD)) command_register = NOCOMMAND;
                    3'h5: begin                         // speak (VSM)
                        SPEN = 1'b1; TALK = 1'b1; DDIS = 1'b0;
                        zpar = 1'b1; uv_zpar = 1'b1; OLDE = 1'b1; OLDP = 1'b1;
                        init_new_frame;
                        command_register = NOCOMMAND;
                    end
                    3'h6: begin                         // speak external
                        begin : clr integer i; for (i = 0; i < 16; i = i + 1) fifo[i] = 8'd0; end
                        fifo_head = 4'd0; fifo_tail = 4'd0; fifo_count = 8'd0; fifo_bits_taken = 3'd0;
                        DDIS = 1'b1;
                        zpar = 1'b1; uv_zpar = 1'b1; OLDE = 1'b1; OLDP = 1'b1;
                        init_new_frame;
                        RDB_flag = 1'b0;
                        command_register = NOCOMMAND;
                    end
                    3'h7: begin chip_reset; command_register = NOCOMMAND; end
                endcase
                update_fifo;
            end
            if (!data_latched) io_ready = 1'b1;
        end
    endtask

    // ---------------- sample sequencer ----------------
    localparam [3:0] S_IDLE = 4'd0, S_PARSE = 4'd1, S_EXC = 4'd2, S_MUL = 4'd3, S_OUT = 4'd4;
    reg  [3:0] state;
    reg  [3:0] pf;          // parse field: 0 energy, 1 repeat, 2 pitch, 3..12 K1..K10
    reg  [2:0] pbit;        // bits left in the field
    reg  [5:0] pval;
    reg        rep_flag;
    reg  [4:0] mstep;       // 0: u10; 1..10: u9..u0; 11..19: x9..x1
    reg        wr_pend, rd_pend, smp_pend;
    reg  [7:0] wr_data;

    function automatic [2:0] field_bits(input [3:0] f);
        case (f)
            4'd0: field_bits = 3'd4;          // energy
            4'd1: field_bits = 3'd1;          // repeat
            4'd2: field_bits = 3'd6;          // pitch
            4'd3, 4'd4: field_bits = 3'd5;    // K1, K2
            4'd5, 4'd6, 4'd7, 4'd8, 4'd9: field_bits = 3'd4;   // K3..K7
            default: field_bits = 3'd3;       // K8..K10
        endcase
    endfunction

    // MAME matrix_multiply: a wrapped to 10 bits, b to 15-bit two's complement (-16384..16383), (a*b)>>9
    reg  signed [9:0]  mm_a;
    reg  signed [14:0] mm_b;
    wire signed [24:0] mm_p = mm_a * mm_b;
    wire signed [23:0] mm_r = {{8{mm_p[24]}}, mm_p[24:9]};

    // excitation helpers
    function automatic [15:0] rng_step20(input [15:0] r);
        integer i;
        reg [15:0] t;
        begin
            t = r;
            for (i = 0; i < 20; i = i + 1)
                t = {t[14:0], t[12] ^ t[3] ^ t[2] ^ t[0]};
            rng_step20 = t;
        end
    endfunction

    function automatic signed [15:0] interp(input signed [15:0] cur, input signed [15:0] tgt, input inh, input [2:0] sh, input zero);
        reg signed [16:0] d;
        begin
            d = inh ? 17'sd0 : ({tgt[15], tgt} - {cur[15], cur});
            interp = zero ? 16'sd0 : (cur + (d >>> sh));
        end
    endfunction

    reg signed [9:0]  ktgt;
    reg signed [15:0] ktgt16;
    reg signed [7:0]  chv;
    reg signed [15:0] wrapped;
    reg signed [15:0] clipped;
    integer k;

    always @(posedge clk) begin
        audio_valid <= 1'b0;
        if (reset) begin
            chip_reset;
            io_ready = 1'b1;
            read_byte_register = 8'd0;
            state <= S_IDLE; wr_pend <= 1'b0; rd_pend <= 1'b0; smp_pend <= 1'b0;
            audio <= -16'sd1; dout <= 8'h60;
        end else begin
            if (wr) begin wr_pend <= 1'b1; wr_data <= din; end
            if (rd) rd_pend <= 1'b1;
            if (ce_sample) smp_pend <= 1'b1;

            case (state)
            S_IDLE: begin
                if (rd_pend && !rd) begin               // MAME status_read(true)
                    rd_pend <= 1'b0;
                    if (RDB_flag) RDB_flag = 1'b0; else irq_pin = 1'b0;
                end else if (wr_pend && !wr) begin
                    wr_pend <= 1'b0;
                    data_write(wr_data);
                end else if (smp_pend && !ce_sample) begin
                    smp_pend <= 1'b0;
                    if (TALKD) begin
                        if (IP == 3'd0 && PC == 4'd12 && subcycle == 2'd1) begin
                            IP = 3'd0;                  // reload_table[0] (5220)
                            uv_zpar = 1'b0; zpar = 1'b0;   // parse_frame()
                            update_fifo;
                            if (DDIS && buffer_empty) state <= S_EXC + 4'd8;   // ranout -> frame done
                            else begin pf <= 4'd0; pbit <= 3'd4; pval <= 6'd0; state <= S_PARSE; end
                        end else begin
                            if (subcycle == 2'd2) begin
                                case (PC)
                                    4'd0: begin
                                        if (IP == 3'd0) pitch_zero = 1'b0;
                                        cur_e = interp(cur_e, {6'd0, t5220_energy(new_e_idx)}, inhibit && IP != 3'd0, t5220_interp(IP), zpar);
                                    end
                                    4'd1: cur_p = interp(cur_p, {6'd0, t5220_pitch(new_p_idx)}, inhibit && IP != 3'd0, t5220_interp(IP), zpar);
                                    4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8, 4'd9, 4'd10, 4'd11: begin
                                        ktgt = t5220_k(PC - 4'd2, new_k_idx[PC-2]);
                                        ktgt16 = ktgt;                       // sign-extends (both signed)
                                        cur_k[PC-2] = interp(cur_k[PC-2], ktgt16, inhibit && IP != 3'd0, t5220_interp(IP), (PC < 4'd6) ? zpar : uv_zpar);
                                    end
                                    default: ;
                                endcase
                            end
                            state <= S_EXC;
                        end
                    end else begin
                        // TALKD == 0: counters only, output -1
                        subcycle = subcycle + 2'd1;
                        if (subcycle == 2'd2 && PC == 4'd12) begin
                            if (IP == 3'd7) begin
                                TALKD = TALK;
                                update_fifo;
                                if (!TALK && SPEN) TALK = 1'b1;
                            end
                            subcycle = 2'd1; PC = 4'd0; IP = IP + 3'd1;
                        end else if (subcycle == 2'd3) begin
                            subcycle = 2'd1; PC = PC + 4'd1;
                        end
                        audio <= -16'sd1; audio_valid <= 1'b1;
                    end
                end
            end

            // ---- parse_frame(): one FIFO bit per clock ----
            S_PARSE: begin
                read_bit;
                begin : pstep
                    reg [5:0] v;
                    v = {pval[4:0], rb_bit};
                    if (pbit != 3'd1) begin
                        pval <= v; pbit <= pbit - 3'd1;
                    end else begin
                        // field complete
                        case (pf)
                            4'd0: begin
                                new_e_idx = v[3:0];
                                update_fifo;
                                if (DDIS && buffer_empty) state <= S_EXC + 4'd8;
                                else if (v[3:0] == 4'd0 || v[3:0] == 4'd15) state <= S_EXC + 4'd8;
                                else begin pf <= 4'd1; pbit <= 3'd1; pval <= 6'd0; end
                            end
                            4'd1: begin rep_flag <= v[0]; pf <= 4'd2; pbit <= 3'd6; pval <= 6'd0; end
                            4'd2: begin
                                new_p_idx = v;
                                uv_zpar = (v == 6'd0);
                                update_fifo;
                                if (DDIS && buffer_empty) state <= S_EXC + 4'd8;
                                else if (rep_flag) state <= S_EXC + 4'd8;
                                else begin pf <= 4'd3; pbit <= 3'd5; pval <= 6'd0; end
                            end
                            default: begin
                                new_k_idx[pf-4'd3] = v[4:0];
                                update_fifo;
                                if (DDIS && buffer_empty) state <= S_EXC + 4'd8;
                                else if (pf == 4'd6 && new_p_idx == 6'd0) state <= S_EXC + 4'd8;   // unvoiced: 4 K's only
                                else if (pf == 4'd12) state <= S_EXC + 4'd8;
                                else begin pf <= pf + 4'd1; pbit <= field_bits(pf + 4'd1); pval <= 6'd0; end
                            end
                        endcase
                    end
                end
            end

            // ---- after parse_frame(): stop flag + interpolation inhibit (encoded as S_EXC+8) ----
            S_EXC + 4'd8: begin
                if (new_e_idx == 4'd15) begin TALK = 1'b0; SPEN = 1'b0; update_fifo; end
                inhibit = (!OLDP && (new_p_idx == 6'd0)) || (OLDP && (new_p_idx != 6'd0))
                       || (OLDE && (new_e_idx != 4'd0)) || (OLDP && (new_e_idx == 4'd0));
                state <= S_EXC;
            end

            // ---- excitation + LFSR, then the lattice ----
            S_EXC: begin
                if (OLDP) exc = RNG[0] ? -16'sd64 : 16'sd64;          // ~0x3F / 0x40
                else begin
                    chv = t5220_chirp((pitch_count >= 9'd51) ? 6'd51 : pitch_count[5:0]);
                    exc = chv;                                        // (int8_t) chirptable[], sign-extends
                end
                RNG = rng_step20(RNG);
                mm_a <= prev_e[9:0];
                mm_b <= {exc[8:0], 6'd0};                             // (excitation << 6), 15-bit wrap
                mstep <= 5'd0;
                state <= S_MUL;
            end

            S_MUL: begin
                // consume the product of the operands loaded last clock, load the next pair
                if (mstep == 5'd0)
                    u[10] = mm_r;
                else if (mstep <= 5'd10)
                    u[10-mstep] = u[11-mstep] - mm_r;                 // u[i] = u[i+1] - k[i]*x[i]
                else
                    x[20-mstep] = x[19-mstep] + mm_r;                 // x[i] = x[i-1] + k[i-1]*u[i-1]
                if (mstep < 5'd10) begin                              // next: k[9-mstep] * x[9-mstep]
                    mm_a <= cur_k[9-mstep][9:0];
                    mm_b <= x[9-mstep][14:0];
                end else if (mstep < 5'd19) begin                     // next: k[8-(mstep-10)] * u[8-(mstep-10)]
                    mm_a <= cur_k[18-mstep][9:0];
                    mm_b <= u[18-mstep][14:0];
                end
                if (mstep == 5'd19) begin
                    x[0] = u[0];
                    prev_e = cur_e;
                    state <= S_OUT;
                end
                mstep <= mstep + 5'd1;
            end

            S_OUT: begin
                // force to 14 bits (wrap), then clip_analog
                wrapped = {{2{u[0][13]}}, u[0][13:0]};
                if (wrapped > 16'sd2047)       clipped = 16'sd2047;
                else if (wrapped < -16'sd2048) clipped = -16'sd2048;
                else                           clipped = wrapped;
                clipped = clipped & ~16'sh000F;
                audio <= (clipped <<< 4) | ((clipped & 16'sh07F0) >>> 3) | ((clipped & 16'sh0400) >>> 10);
                audio_valid <= 1'b1;
                // update counts
                subcycle = subcycle + 2'd1;
                if (subcycle == 2'd2 && PC == 4'd12) begin
                    if (IP == 3'd7 && inhibit) pitch_zero = 1'b1;
                    if (IP == 3'd7) begin
                        OLDE = (new_e_idx == 4'd0);
                        OLDP = (new_p_idx == 6'd0);
                        TALKD = TALK;
                        update_fifo;
                        if (!TALK && SPEN) TALK = 1'b1;
                    end
                    subcycle = 2'd1; PC = 4'd0; IP = IP + 3'd1;
                end else if (subcycle == 2'd3) begin
                    subcycle = 2'd1; PC = PC + 4'd1;
                end
                pitch_count = pitch_count + 9'd1;
                if (($signed({7'd0, pitch_count}) >= cur_p) || pitch_zero) pitch_count = 9'd0;
                state <= S_IDLE;
            end

            default: state <= S_IDLE;
            endcase

            dout <= RDB_flag ? read_byte_register : {SPEN | TALKD, buffer_low, buffer_empty, 5'd0};
        end
    end

endmodule
