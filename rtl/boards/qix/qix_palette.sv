// qix_palette.sv — 1024-byte Palette RAM + RRGGBBII-to-RGB Decode
//
// 4 banks × 256 entries. Bank select written at $8800 bits[1:0].
// CPU accesses $9000-$93FF (10-bit address).
// Display pipeline: pixel_index → palette lookup → LUT → RGB outputs.

module qix_palette (
    input             clk,

    // CPU access ($9000-$93FF, addr = cpu_addr[9:0])
    input  [9:0]      cpu_addr,
    input             cpu_we,
    input  [7:0]      cpu_din,
    output [7:0]      cpu_dout,

    // Bank select (written at $8800, bits [1:0])
    input             bank_we,
    input  [1:0]      bank_din,

    // Display pipeline
    input  [7:0]      pixel_index,   // raw VRAM byte from scanout
    output [7:0]      rgb_r,
    output [7:0]      rgb_g,
    output [7:0]      rgb_b,

    // Palette RAM storage: RAM pool block 4 in the dispatcher
    output ram_pool_pkg::pram_i_t pr_i,
    input  ram_pool_pkg::pram_o_t pr_o
);

// ---------------------------------------------------------------------------
// 1024-byte palette RAM — RAM pool block 4 (port A = CPU, port B = display lookup)
// ---------------------------------------------------------------------------

// Active palette bank register
reg [1:0] palette_bank;

always @(posedge clk) begin
    if (bank_we)
        palette_bank <= bank_din;
end

// CPU read/write (synchronous)
assign pr_i.a_addr = {6'd0, cpu_addr};
assign pr_i.a_d    = cpu_din;
assign pr_i.a_we   = cpu_we;
assign cpu_dout    = pr_o.a_q;

// Display lookup: registered read from palette RAM
wire [7:0] pal_out;
assign pr_i.b_addr = {6'd0, palette_bank, pixel_index};
assign pr_i.b_d    = 8'd0;
assign pr_i.b_we   = 1'b0;
assign pal_out     = pr_o.b_q;

// ---------------------------------------------------------------------------
// RRGGBBII → RGB conversion (qix.cpp)
// Index = {color_value[1:0], intensity[1:0]}
// ---------------------------------------------------------------------------
reg [7:0] lut [0:15];
initial begin
    lut[0]  = 8'h00; lut[1]  = 8'h12; lut[2]  = 8'h24; lut[3]  = 8'h49;
    lut[4]  = 8'h12; lut[5]  = 8'h24; lut[6]  = 8'h49; lut[7]  = 8'h92;
    lut[8]  = 8'h5B; lut[9]  = 8'h6D; lut[10] = 8'h92; lut[11] = 8'hDB;
    lut[12] = 8'h7F; lut[13] = 8'h91; lut[14] = 8'hB6; lut[15] = 8'hFF;
end

assign rgb_r = lut[{pal_out[7:6], pal_out[1:0]}];
assign rgb_g = lut[{pal_out[5:4], pal_out[1:0]}];
assign rgb_b = lut[{pal_out[3:2], pal_out[1:0]}];

endmodule
