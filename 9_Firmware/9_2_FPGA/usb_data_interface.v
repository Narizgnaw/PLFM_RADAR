module usb_data_interface (
    input wire clk,              // Main clock (100MHz recommended)
    input wire reset_n,
    input wire ft601_reset_n,    // FT601-domain synchronized reset
    
    // Radar data inputs
    input wire [31:0] range_profile,
    input wire range_valid,
    input wire [15:0] doppler_real,
    input wire [15:0] doppler_imag,
    input wire doppler_valid,
    input wire cfar_detection,
    input wire cfar_valid,
    
    // FT601 Interface (Slave FIFO mode)
    // Data bus
    inout wire [31:0] ft601_data,    // 32-bit bidirectional data bus
    output reg [3:0] ft601_be,       // Byte enable (4 lanes for 32-bit mode)
    
    // Control signals
    output reg ft601_txe_n,          // Transmit enable (active low)
    output reg ft601_rxf_n,          // Receive enable (active low)
    input wire ft601_txe,             // Transmit FIFO empty
    input wire ft601_rxf,             // Receive FIFO full
    output reg ft601_wr_n,            // Write strobe (active low)
    output reg ft601_rd_n,            // Read strobe (active low)
    output reg ft601_oe_n,            // Output enable (active low)
    output reg ft601_siwu_n,          // Send immediate / Wakeup
    
    // FIFO flags
    input wire [1:0] ft601_srb,       // Selected read buffer
    input wire [1:0] ft601_swb,       // Selected write buffer
    
    // Clock
    output wire ft601_clk_out,        // Output clock to FT601 (forwarded via ODDR)
    input wire ft601_clk_in,          // Clock from FT601 (60/100MHz)
    
    // ========== HOST COMMAND OUTPUTS (Gap 4: USB Read Path) ==========
    // These outputs are registered in the ft601_clk domain and must be
    // CDC-synchronized by the consumer (radar_system_top.v) before use
    // in the clk_100m domain.
    //
    // Command word format: {opcode[7:0], addr[7:0], value[15:0]}
    //   0x01 = Set radar mode (value[1:0] -> mode_controller mode)
    //   0x02 = Single chirp trigger (value ignored, pulse cmd_valid)
    //   0x03 = Set CFAR threshold (value[15:0] -> threshold)
    //   0x04 = Set stream control (value[2:0] -> enable range/doppler/cfar)
    //   0xFF = Status request (triggers status response packet)
    output reg [31:0] cmd_data,      // Last received command word
    output reg cmd_valid,            // Pulse: new command received (ft601_clk domain)
    output reg [7:0] cmd_opcode,     // Decoded opcode for convenience
    output reg [7:0] cmd_addr,       // Decoded register address
    output reg [15:0] cmd_value      // Decoded value
);

// USB packet structure (same as before)
localparam HEADER = 8'hAA;
localparam FOOTER = 8'h55;

// FT601 configuration
localparam FT601_DATA_WIDTH = 32;
localparam FT601_BURST_SIZE = 512;    // Max burst size in bytes

// ============================================================================
// WRITE FSM State definitions (Verilog-2001 compatible)
// ============================================================================
localparam [2:0] IDLE                = 3'd0,
                 SEND_HEADER         = 3'd1,
                 SEND_RANGE_DATA     = 3'd2,
                 SEND_DOPPLER_DATA   = 3'd3,
                 SEND_DETECTION_DATA = 3'd4,
                 SEND_FOOTER         = 3'd5,
                 WAIT_ACK            = 3'd6;

reg [2:0] current_state;
reg [7:0] byte_counter;
reg [31:0] data_buffer;
reg [31:0] ft601_data_out;
reg ft601_data_oe;  // Output enable for bidirectional data bus

// ============================================================================
// READ FSM State definitions (Gap 4: USB Read Path)
// ============================================================================
// FT601 245 synchronous FIFO read protocol:
//   1. Host has data available: RXF_N = 0 (active low)
//   2. FPGA asserts OE_N = 0 (bus turnaround: FT601 starts driving data bus)
//   3. Wait 1 cycle for bus turnaround settling
//   4. FPGA asserts RD_N = 0 (start reading: data valid on each posedge)
//   5. Sample ft601_data[31:0] while RD_N = 0 and RXF_N = 0
//   6. Deassert RD_N = 1, then OE_N = 1
//
// The read FSM only activates when the write FSM is IDLE and RXF indicates
// data is available. This prevents bus contention between TX and RX.
localparam [2:0] RD_IDLE      = 3'd0,  // Waiting for RXF
                 RD_OE_ASSERT = 3'd1,  // Assert OE_N=0, wait turnaround
                 RD_READING   = 3'd2,  // Assert RD_N=0, sample data
                 RD_DEASSERT  = 3'd3,  // Deassert RD_N, then OE_N
                 RD_PROCESS   = 3'd4;  // Process received command word

reg [2:0] read_state;
reg [31:0] rx_data_captured;  // Data word read from host

// ========== CDC INPUT SYNCHRONIZERS (clk domain -> ft601_clk_in domain) ==========
// The valid signals arrive from clk_100m but the state machine runs on ft601_clk_in.
// Even though both are 100 MHz, they are asynchronous clocks and need synchronization.

// 2-stage synchronizers for valid signals
(* ASYNC_REG = "TRUE" *) reg [1:0] range_valid_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] doppler_valid_sync;
(* ASYNC_REG = "TRUE" *) reg [1:0] cfar_valid_sync;

// Delayed versions of sync[1] for proper edge detection
reg range_valid_sync_d;
reg doppler_valid_sync_d;
reg cfar_valid_sync_d;

// Holding registers: data captured in SOURCE domain (clk_100m) when valid
// asserts, then read by ft601 domain after synchronized valid edge.
// This is safe because the data is stable for the entire time the valid
// pulse is being synchronized (2+ ft601_clk cycles).
reg [31:0] range_profile_hold;
reg [15:0] doppler_real_hold;
reg [15:0] doppler_imag_hold;
reg cfar_detection_hold;

// Source-domain holding registers (clk domain)
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        range_profile_hold <= 32'd0;
        doppler_real_hold  <= 16'd0;
        doppler_imag_hold  <= 16'd0;
        cfar_detection_hold <= 1'b0;
    end else begin
        if (range_valid)
            range_profile_hold <= range_profile;
        if (doppler_valid) begin
            doppler_real_hold <= doppler_real;
            doppler_imag_hold <= doppler_imag;
        end
        if (cfar_valid)
            cfar_detection_hold <= cfar_detection;
    end
end

// FT601-domain captured data (sampled from holding regs on sync'd edge)
reg [31:0] range_profile_cap;
reg [15:0] doppler_real_cap;
reg [15:0] doppler_imag_cap;
reg cfar_detection_cap;

wire range_valid_ft;
wire doppler_valid_ft;
wire cfar_valid_ft;

always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n) begin
        range_valid_sync   <= 2'b00;
        doppler_valid_sync <= 2'b00;
        cfar_valid_sync    <= 2'b00;
        range_valid_sync_d   <= 1'b0;
        doppler_valid_sync_d <= 1'b0;
        cfar_valid_sync_d    <= 1'b0;
        range_profile_cap  <= 32'd0;
        doppler_real_cap   <= 16'd0;
        doppler_imag_cap   <= 16'd0;
        cfar_detection_cap <= 1'b0;
    end else begin
        // Synchronize valid strobes (2-stage sync chain)
        range_valid_sync   <= {range_valid_sync[0],   range_valid};
        doppler_valid_sync <= {doppler_valid_sync[0], doppler_valid};
        cfar_valid_sync    <= {cfar_valid_sync[0],    cfar_valid};

        // Delayed version of sync[1] for edge detection
        range_valid_sync_d   <= range_valid_sync[1];
        doppler_valid_sync_d <= doppler_valid_sync[1];
        cfar_valid_sync_d    <= cfar_valid_sync[1];

        // Capture data on rising edge of FULLY SYNCHRONIZED valid (sync[1])
        // Data in holding regs is stable by the time sync[1] rises (2+ cycles)
        if (range_valid_sync[1] && !range_valid_sync_d)
            range_profile_cap <= range_profile_hold;
        if (doppler_valid_sync[1] && !doppler_valid_sync_d) begin
            doppler_real_cap <= doppler_real_hold;
            doppler_imag_cap <= doppler_imag_hold;
        end
        if (cfar_valid_sync[1] && !cfar_valid_sync_d)
            cfar_detection_cap <= cfar_detection_hold;
    end
end

// Rising-edge detect on FULLY SYNCHRONIZED valid (sync[1], not sync[0])
// This provides full 2-stage metastability protection before use.
assign range_valid_ft   = range_valid_sync[1]   && !range_valid_sync_d;
assign doppler_valid_ft = doppler_valid_sync[1]  && !doppler_valid_sync_d;
assign cfar_valid_ft    = cfar_valid_sync[1]     && !cfar_valid_sync_d;

// FT601 data bus direction control
assign ft601_data = ft601_data_oe ? ft601_data_out : 32'hzzzz_zzzz;

always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n) begin
        current_state <= IDLE;
        read_state <= RD_IDLE;
        byte_counter <= 0;
        ft601_data_out <= 0;
        ft601_data_oe <= 0;
        ft601_be <= 4'b1111;  // All bytes enabled for 32-bit mode
        ft601_txe_n <= 1;
        ft601_rxf_n <= 1;
        ft601_wr_n <= 1;
        ft601_rd_n <= 1;
        ft601_oe_n <= 1;
        ft601_siwu_n <= 1;
        rx_data_captured <= 32'd0;
        cmd_data <= 32'd0;
        cmd_valid <= 1'b0;
        cmd_opcode <= 8'd0;
        cmd_addr <= 8'd0;
        cmd_value <= 16'd0;
        // NOTE: ft601_clk_out is driven by the clk-domain always block below.
        // Do NOT assign it here (ft601_clk_in domain) — causes multi-driven net.
    end else begin
        // Default: clear one-shot signals
        cmd_valid <= 1'b0;

        // ================================================================
        // READ FSM — host-to-FPGA command path (Gap 4)
        //
        // The read FSM takes priority over write when both could activate.
        // It only starts when the write FSM is IDLE and ft601_rxf
        // indicates data from host is available.
        // ================================================================
        case (read_state)
            RD_IDLE: begin
                // Only start reading if write FSM is idle and host has data.
                // ft601_rxf active-low: 0 means data available from host.
                if (current_state == IDLE && !ft601_rxf) begin
                    ft601_oe_n <= 1'b0;     // Assert OE: tell FT601 to drive bus
                    ft601_data_oe <= 1'b0;  // FPGA releases bus (FT601 drives)
                    read_state <= RD_OE_ASSERT;
                end
            end

            RD_OE_ASSERT: begin
                // 1-cycle turnaround: OE_N asserted, bus settling.
                // FT601 spec requires 1 clock of OE_N before RD_N assertion.
                if (!ft601_rxf) begin
                    ft601_rd_n <= 1'b0;     // Assert RD: start reading
                    read_state <= RD_READING;
                end else begin
                    // Host withdrew data — abort
                    ft601_oe_n <= 1'b1;
                    read_state <= RD_IDLE;
                end
            end

            RD_READING: begin
                // Data is valid on ft601_data. Sample it.
                // For now we read a single 32-bit command word per transaction.
                rx_data_captured <= ft601_data;
                ft601_rd_n <= 1'b1;         // Deassert RD
                read_state <= RD_DEASSERT;
            end

            RD_DEASSERT: begin
                // Deassert OE_N (1 cycle after RD_N deasserted)
                ft601_oe_n <= 1'b1;
                read_state <= RD_PROCESS;
            end

            RD_PROCESS: begin
                // Decode the received command word and pulse cmd_valid.
                // Format: {opcode[31:24], addr[23:16], value[15:0]}
                cmd_data   <= rx_data_captured;
                cmd_opcode <= rx_data_captured[31:24];
                cmd_addr   <= rx_data_captured[23:16];
                cmd_value  <= rx_data_captured[15:0];
                cmd_valid  <= 1'b1;
                read_state <= RD_IDLE;
            end

            default: read_state <= RD_IDLE;
        endcase

        // ================================================================
        // WRITE FSM — FPGA-to-host data streaming (existing)
        //
        // Only operates when read FSM is idle (no bus contention).
        // ================================================================
        if (read_state == RD_IDLE) begin
            case (current_state)
                IDLE: begin
                    ft601_wr_n <= 1;
                    ft601_data_oe <= 0;  // Release data bus
                    if (range_valid_ft || doppler_valid_ft || cfar_valid_ft) begin
                        // Don't start write if a read is about to begin
                        if (ft601_rxf) begin  // rxf=1 means no host data pending
                            current_state <= SEND_HEADER;
                            byte_counter <= 0;
                        end
                    end
                end
                
                SEND_HEADER: begin
                    if (!ft601_txe) begin  // FT601 TX FIFO not empty
                        ft601_data_oe <= 1;
                        ft601_data_out <= {24'b0, HEADER};
                        ft601_be <= 4'b0001;  // Only lower byte valid
                        ft601_wr_n <= 0;     // Assert write strobe
                        current_state <= SEND_RANGE_DATA;
                    end
                end
                
                SEND_RANGE_DATA: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;  // All bytes valid for 32-bit word
                        
                        case (byte_counter)
                            0: ft601_data_out <= range_profile_cap;
                            1: ft601_data_out <= {range_profile_cap[23:0], 8'h00};
                            2: ft601_data_out <= {range_profile_cap[15:0], 16'h0000};
                            3: ft601_data_out <= {range_profile_cap[7:0], 24'h000000};
                        endcase
                        
                        ft601_wr_n <= 0;
                        
                        if (byte_counter == 3) begin
                            byte_counter <= 0;
                            current_state <= SEND_DOPPLER_DATA;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                end
                
                SEND_DOPPLER_DATA: begin
                    if (!ft601_txe && doppler_valid_ft) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b1111;
                        
                        case (byte_counter)
                            0: ft601_data_out <= {doppler_real_cap, doppler_imag_cap};
                            1: ft601_data_out <= {doppler_imag_cap, doppler_real_cap[15:8], 8'h00};
                            2: ft601_data_out <= {doppler_real_cap[7:0], doppler_imag_cap[15:8], 16'h0000};
                            3: ft601_data_out <= {doppler_imag_cap[7:0], 24'h000000};
                        endcase
                        
                        ft601_wr_n <= 0;
                        
                        if (byte_counter == 3) begin
                            byte_counter <= 0;
                            current_state <= SEND_DETECTION_DATA;
                        end else begin
                            byte_counter <= byte_counter + 1;
                        end
                    end
                end
                
                SEND_DETECTION_DATA: begin
                    if (!ft601_txe && cfar_valid_ft) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b0001;
                        ft601_data_out <= {24'b0, 7'b0, cfar_detection_cap};
                        ft601_wr_n <= 0;
                        current_state <= SEND_FOOTER;
                    end
                end
                
                SEND_FOOTER: begin
                    if (!ft601_txe) begin
                        ft601_data_oe <= 1;
                        ft601_be <= 4'b0001;
                        ft601_data_out <= {24'b0, FOOTER};
                        ft601_wr_n <= 0;
                        current_state <= WAIT_ACK;
                    end
                end
                
                WAIT_ACK: begin
                    ft601_wr_n <= 1;
                    ft601_data_oe <= 0;  // Release data bus
                    current_state <= IDLE;
                end
            endcase
        end
    end
end

// ============================================================================
// FT601 clock output forwarding
// ============================================================================
// Forward ft601_clk_in back out via ODDR so that the forwarded clock at the
// pin has the same insertion delay as the data outputs (both go through the
// same BUFG). This makes the output delay analysis relative to the generated
// clock at the pin, where insertion delays cancel.

`ifndef SIMULATION
ODDR #(
    .DDR_CLK_EDGE("OPPOSITE_EDGE"),
    .INIT(1'b0),
    .SRTYPE("SYNC")
) oddr_ft601_clk (
    .Q(ft601_clk_out),
    .C(ft601_clk_in),
    .CE(1'b1),
    .D1(1'b1),
    .D2(1'b0),
    .R(1'b0),
    .S(1'b0)
);
`else
// Simulation: behavioral clock forwarding
reg ft601_clk_out_sim;
always @(posedge ft601_clk_in or negedge ft601_reset_n) begin
    if (!ft601_reset_n)
        ft601_clk_out_sim <= 1'b0;
    else
        ft601_clk_out_sim <= 1'b1;
end
// In simulation, just pass the clock through
assign ft601_clk_out = ft601_clk_in;
`endif

endmodule