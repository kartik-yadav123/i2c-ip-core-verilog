`timescale 1ns/1ps

module tb_i2c_master;

    // ----------------------------------------------------------------
    // Wishbone signals
    // ----------------------------------------------------------------
    reg        wb_clk_i;
    reg        wb_rst_i;
    reg        arst_i;
    reg  [2:0] wb_adr_i;
    reg  [7:0] wb_dat_i;
    wire [7:0] wb_dat_o;
    reg        wb_we_i;
    reg        wb_stb_i;
    reg        wb_cyc_i;
    wire       wb_ack_o;
    wire       wb_inta_o;

    // ----------------------------------------------------------------
    // I2C open-drain lines
    // ----------------------------------------------------------------
    wire scl_pad_o, scl_padoen_o;
    wire sda_pad_o, sda_padoen_o;

    // Pull-up resistor model (tri1 = weak-1 when not driven)
    tri1 scl_wire;
    tri1 sda_wire;

    // Master drives lines when padoen = 0 (active-low output-enable)
    assign scl_wire = (scl_padoen_o == 1'b0) ? scl_pad_o : 1'bz;
    assign sda_wire = (sda_padoen_o == 1'b0) ? sda_pad_o : 1'bz;

    // Slave ACK driver
    reg sda_slave_drive;
    assign sda_wire = sda_slave_drive ? 1'b0 : 1'bz;

    // Feed wires back into DUT
    wire scl_pad_i = scl_wire;
    wire sda_pad_i = sda_wire;

    // ----------------------------------------------------------------
    // DUT instantiation
    // ----------------------------------------------------------------
    i2c_master_top dut (
        .wb_clk_i   (wb_clk_i),
        .wb_rst_i   (wb_rst_i),
        .arst_i     (arst_i),
        .wb_adr_i   (wb_adr_i),
        .wb_dat_i   (wb_dat_i),
        .wb_dat_o   (wb_dat_o),
        .wb_we_i    (wb_we_i),
        .wb_stb_i   (wb_stb_i),
        .wb_cyc_i   (wb_cyc_i),
        .wb_ack_o   (wb_ack_o),
        .wb_inta_o  (wb_inta_o),
        .scl_pad_i  (scl_pad_i),
        .scl_pad_o  (scl_pad_o),
        .scl_padoen_o(scl_padoen_o),
        .sda_pad_i  (sda_pad_i),
        .sda_pad_o  (sda_pad_o),
        .sda_padoen_o(sda_padoen_o)
    );

    // ----------------------------------------------------------------
    // Clock: 50 MHz  (period = 20 ns)
    // ----------------------------------------------------------------
    initial wb_clk_i = 1'b0;
    always  #10 wb_clk_i = ~wb_clk_i;

    // ----------------------------------------------------------------
    // Wishbone read/write tasks  (drive AFTER negedge to avoid races)
    // ----------------------------------------------------------------

    // Write: addr + data, wait for ACK, de-assert strobe
    task wb_write;
        input [2:0] addr;
        input [7:0] data;
        integer timeout;
        begin
            @(negedge wb_clk_i);        // drive safely after falling edge
            wb_adr_i = addr;
            wb_dat_i = data;
            wb_we_i  = 1'b1;
            wb_stb_i = 1'b1;
            wb_cyc_i = 1'b1;
            timeout  = 0;
            // Wait for ACK with a time-out guard (200 cycles)
            @(posedge wb_clk_i);
            while (!wb_ack_o && timeout < 200) begin
                @(posedge wb_clk_i);
                timeout = timeout + 1;
            end
            if (timeout >= 200)
                $display("[WB_WRITE] WARNING: ACK timeout addr=%0h data=%0h", addr, data);
            @(negedge wb_clk_i);
            wb_stb_i = 1'b0;
            wb_cyc_i = 1'b0;
            wb_we_i  = 1'b0;
        end
    endtask

    // Read: addr, wait for ACK, latch result
    task wb_read;
        input  [2:0] addr;
        output [7:0] rdata;
        integer timeout;
        begin
            @(negedge wb_clk_i);
            wb_adr_i = addr;
            wb_we_i  = 1'b0;
            wb_stb_i = 1'b1;
            wb_cyc_i = 1'b1;
            timeout  = 0;
            @(posedge wb_clk_i);
            while (!wb_ack_o && timeout < 200) begin
                @(posedge wb_clk_i);
                timeout = timeout + 1;
            end
            rdata = wb_dat_o;
            @(negedge wb_clk_i);
            wb_stb_i = 1'b0;
            wb_cyc_i = 1'b0;
        end
    endtask

    // Poll Command Register until TIP (bit 1) clears, with timeout
    task wait_tip_clear;
        integer timeout;
        reg [7:0] status;
        begin
            timeout = 0;
            status  = 8'hFF;
            while (status[1] && timeout < 5000) begin
                wb_read(3'd4, status);    // reg 4 = Status Register
                timeout = timeout + 1;
            end
            if (timeout >= 5000)
                $display("[WAIT_TIP] WARNING: TIP never cleared");
        end
    endtask

    // ----------------------------------------------------------------
    // Slave ACK generator
    //   Counts SCL rising edges; on the 9th falling edge pulls SDA low
    //   for exactly one SCL cycle to signal ACK.
    // ----------------------------------------------------------------
    integer bit_cnt;
    initial begin
        sda_slave_drive = 1'b0;
        bit_cnt = 0;
    end

    always @(posedge scl_wire)
        bit_cnt = bit_cnt + 1;

    always @(negedge scl_wire) begin
        if (bit_cnt == 8) begin
            sda_slave_drive = 1'b1;   // pull SDA low  = ACK
            $display("[SLAVE]  ACK sent at time %0t", $time);
        end else if (bit_cnt == 9) begin
            sda_slave_drive = 1'b0;   // release SDA
            bit_cnt = 0;              // reset for next byte
        end
    end

    // ----------------------------------------------------------------
    // Waveform dump
    // ----------------------------------------------------------------
    initial begin
        $dumpfile("i2c.vcd");
        $dumpvars(0, tb_i2c_master);
    end

    // ----------------------------------------------------------------
    // Register map monitor (prints whenever wb_ack_o fires)
    // ----------------------------------------------------------------
    always @(posedge wb_ack_o) begin
        if (wb_we_i)
            $display("[WB]  WRITE  addr=%0h  data=0x%02h  @%0t ns",
                     wb_adr_i, wb_dat_i, $time);
        else
            $display("[WB]  READ   addr=%0h  data=0x%02h  @%0t ns",
                     wb_adr_i, wb_dat_o, $time);
    end

    // ----------------------------------------------------------------
    // I2C line monitor
    // ----------------------------------------------------------------
    always @(negedge sda_wire) if (scl_wire) $display("[I2C] START condition  @%0t ns", $time);
    always @(posedge sda_wire) if (scl_wire) $display("[I2C] STOP  condition  @%0t ns", $time);

    // ----------------------------------------------------------------
    // Main test sequence
    // ----------------------------------------------------------------
    reg [7:0] rd_status;

    initial begin
        // --------------------------------------------------
        // Initialise all bus signals
        // --------------------------------------------------
        arst_i   = 1'b1;    // arst_i is ACTIVE HIGH in most opencores builds
        wb_rst_i = 1'b1;    // synchronous reset active
        wb_we_i  = 1'b0;
        wb_stb_i = 1'b0;
        wb_cyc_i = 1'b0;
        wb_adr_i = 3'd0;
        wb_dat_i = 8'd0;

        repeat(5) @(posedge wb_clk_i);

        // --------------------------------------------------
        // FIX: arst_i polarity
        //   opencores i2c_master_top uses arst_i as
        //   asynchronous reset: reset when arst_i == ARST_LVL
        //   Default ARST_LVL parameter = 1'b0 in the RTL,
        //   meaning arst_i=0 ? in reset, arst_i=1 ? running.
        //   Keep arst_i=1 (release reset) and de-assert wb_rst_i.
        // --------------------------------------------------
        wb_rst_i = 1'b0;    // release synchronous reset
        // arst_i stays 1  (= not-reset when ARST_LVL=0)

        repeat(3) @(posedge wb_clk_i);

        // --------------------------------------------------
        // Configure prescaler
        //   I2C_SCL = wb_clk / (5 * (prescale + 1))
        //   For 50 MHz clock, prescale = 49 ? ~200 kHz (standard)
        //   prescale = 9  ? ~1 MHz  (fast for sim)
        //   Using 9 here for faster simulation
        // --------------------------------------------------
        $display("=== Configuring prescaler ===");
        wb_write(3'd0, 8'd9);     // PRERlo = 9
        wb_write(3'd1, 8'd0);     // PRERhi = 0

        // Read back to confirm (sanity check)
        wb_read(3'd0, rd_status);
        $display("[CFG]  PRERlo readback = 0x%02h (expect 0x09)", rd_status);

        // --------------------------------------------------
        // Enable core  (CTR register bit7 = EN)
        // --------------------------------------------------
        wb_write(3'd2, 8'h80);
        $display("=== Core enabled ===");

        // --------------------------------------------------
        // WRITE transaction:  device address 0xA0 + data 0xA5
        // --------------------------------------------------
        $display("=== START WRITE transaction ===");

        // Step 1 ? load slave address byte (0xA0 = addr 0x50, R/W=0)
        wb_write(3'd3, 8'hA0);            // TXR = 0xA0
        // Step 2 ? issue START + WRITE command
        wb_write(3'd4, 8'h90);            // CR  = STA | WR
        wait_tip_clear;                   // wait until byte is shifted out

        // Read status to check ACK received (SR bit 7 = RxACK, 0=ACK)
        wb_read(3'd4, rd_status);
        if (!rd_status[7])
            $display("[STATUS] Address byte ACKed  (SR=0x%02h)", rd_status);
        else
            $display("[STATUS] Address byte NACKed (SR=0x%02h)", rd_status);

        // Step 3 ? load data byte
        wb_write(3'd3, 8'hA5);            // TXR = 0xA5
        // Step 4 ? WRITE + STOP
        wb_write(3'd4, 8'h50);            // CR  = STO | WR
        wait_tip_clear;

        wb_read(3'd4, rd_status);
        if (!rd_status[7])
            $display("[STATUS] Data byte ACKed  (SR=0x%02h)", rd_status);
        else
            $display("[STATUS] Data byte NACKed (SR=0x%02h)", rd_status);

        $display("=== WRITE transaction DONE ===");

        // --------------------------------------------------
        // READ transaction:  re-address device, read 1 byte
        // --------------------------------------------------
        $display("=== START READ transaction ===");

        // Load slave address with R/W=1 (read)
        wb_write(3'd3, 8'hA1);            // TXR = 0xA1
        wb_write(3'd4, 8'h90);            // CR  = STA | WR  (send address)
        wait_tip_clear;

        wb_read(3'd4, rd_status);
        $display("[STATUS] Read address sent (SR=0x%02h)", rd_status);

        // Issue READ + NACK (last byte) + STOP
        wb_write(3'd4, 8'h68);            // CR  = RD | NACK | STO
        wait_tip_clear;

        // Read received data
        wb_read(3'd3, rd_status);
        $display("[RXD]  Received byte = 0x%02h", rd_status);

        $display("=== ALL TESTS DONE ===");
        repeat(10) @(posedge wb_clk_i);
        $finish;
    end

endmodule
