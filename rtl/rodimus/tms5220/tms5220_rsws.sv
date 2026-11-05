//  Based on MAME tms5220.cpp (Frank Palazzolo, Aaron Giles, Jonathan Gevaryahu, Raphael Nabet, Couriersud, Michael Zapf, BSD-3-Clause)

module tms5220_rsws #(
    parameter int CLK_HZ = 22578000,
    parameter int TMS_HZ = 640000
)
(
    input  logic        clk,
    input  logic        reset,
    input  logic        pause,
    input  logic        rs_n,
    input  logic        ws_n,
    input  logic  [7:0] d_in,
    output logic  [7:0] d_out,
    output logic        ready_n,
    output logic        int_n,
    output logic signed [15:0] audio
);

localparam logic [31:0] SMP_MOD = CLK_HZ * 80;
logic [31:0] smp_acc;
logic        ce_smp;
always_ff @(posedge clk) begin
    if (reset) begin
        smp_acc <= 32'd0; ce_smp <= 1'b0;
    end else begin
        ce_smp <= 1'b0;
        if (!pause) begin
            if (smp_acc + TMS_HZ >= SMP_MOD) begin smp_acc <= smp_acc + TMS_HZ - SMP_MOD; ce_smp <= 1'b1; end
            else smp_acc <= smp_acc + TMS_HZ;
        end
    end
end

logic       c_wr, c_rd;
logic [7:0] c_din, c_dout;
logic       c_ready_n;

tms5220 tms
(
    .clk(clk), .reset(reset), .ce_sample(ce_smp),
    .wr(c_wr), .din(c_din), .rd(c_rd), .dout(c_dout),
    .int_n(int_n), .ready_n(c_ready_n),
    .audio(audio), .audio_valid()
);

localparam int          W_INC_I = TMS_HZ;
localparam int          W_THR_I = 16 * CLK_HZ;
localparam int          R_THR_I = 13 * CLK_HZ;
localparam logic [29:0] W_INC   = W_INC_I[29:0];
localparam logic [29:0] W_THR   = W_THR_I[29:0];
localparam logic [29:0] R_INC   = 30'd1000000;
localparam logic [29:0] R_THR   = R_THR_I[29:0];

logic [1:0]  rsws;
logic        io_ready, tmr_on, tmr_rd, rd_valid;
logic [29:0] tmr_acc;
logic [7:0]  rlatch;

assign ready_n = ~io_ready;
assign d_out   = (rsws == 2'b01) ? rlatch : 8'hFF;

always_ff @(posedge clk) begin : pin_fsm
    logic [1:0] s;
    logic       arm, arm_rd, ff;
    if (reset) begin
        rsws <= 2'b11; io_ready <= 1'b1; tmr_on <= 1'b0; tmr_rd <= 1'b0; tmr_acc <= '0;
        rlatch <= 8'hFF; rd_valid <= 1'b0; c_wr <= 1'b0; c_rd <= 1'b0; c_din <= 8'h00;
    end else begin
        c_wr <= 1'b0;
        c_rd <= 1'b0;
        s = rsws; arm = 1'b0; arm_rd = 1'b0; ff = 1'b0;

        if (rs_n != s[1]) begin
            s[1] = rs_n;
            if (s == 2'b11)              ff = 1'b1;
            else if (s != 2'b00 && !rs_n) begin arm = 1'b1; arm_rd = 1'b1; end
        end
        if (ws_n != s[0]) begin
            s[0] = ws_n;
            if (s == 2'b11)              ff = 1'b1;
            else if (s != 2'b00 && !ws_n) begin arm = 1'b1; arm_rd = 1'b0; end
        end
        rsws <= s;
        if (s != 2'b01) rd_valid <= 1'b0;

        if (arm) begin
            io_ready <= 1'b0;
            tmr_on   <= 1'b1;
            tmr_rd   <= arm_rd;
            tmr_acc  <= '0;
        end else if (tmr_on && !pause) begin
            if (tmr_acc + (tmr_rd ? R_INC : W_INC) >= (tmr_rd ? R_THR : W_THR)) begin
                tmr_on <= 1'b0;
                case (s)
                    2'b10: begin
                        if (c_ready_n) begin
                            tmr_on <= 1'b1; tmr_rd <= 1'b0; tmr_acc <= '0;
                        end else begin
                            c_wr <= 1'b1; c_din <= d_in; io_ready <= 1'b1;
                        end
                    end
                    2'b01: begin
                        rlatch <= c_dout; c_rd <= 1'b1; io_ready <= 1'b1; rd_valid <= 1'b1;
                        ff = 1'b0;
                    end
                    default: io_ready <= 1'b1;
                endcase
            end else
                tmr_acc <= tmr_acc + (tmr_rd ? R_INC : W_INC);
        end

        if (ff)                            rlatch <= 8'hFF;
        else if (rd_valid && s == 2'b01)   rlatch <= c_dout;
    end
end

endmodule
