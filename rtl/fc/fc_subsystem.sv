// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.


`include "soc_bus_defines.sv"

module fc_subsystem #(
    parameter CORE_TYPE           = 0,
    parameter USE_FPU             = 1,
    parameter USE_HWPE            = 1,
    parameter N_EXT_PERF_COUNTERS = 1,
    parameter EVENT_ID_WIDTH      = 8,
    parameter PER_ID_WIDTH        = 32,
    parameter NB_HWPE_PORTS       = 4,
    parameter PULP_SECURE         = 1,
    parameter TB_RISCV            = 0,
    parameter CORE_ID             = 4'h0,
    parameter CLUSTER_ID          = 6'h1F
)
(
    input  logic                      clk_i,
    input  logic                      rst_ni,
    input  logic                      test_en_i,

    XBAR_TCDM_BUS.Master              l2_data_master,
    XBAR_TCDM_BUS.Master              l2_instr_master,
    XBAR_TCDM_BUS.Master              l2_hwpe_master [NB_HWPE_PORTS-1:0],
`ifdef QUENTIN_SCM
    UNICAD_MEM_BUS_32.Master          scm_l2_data_master,
    UNICAD_MEM_BUS_32.Master          scm_l2_instr_master,
`endif
    APB_BUS.Slave                     apb_slave_eu,
    APB_BUS.Slave                     apb_slave_hwpe,

    input  logic                      fetch_en_i,
    input  logic [31:0]               boot_addr_i,
    input  logic                      debug_req_i,

    input  logic                      event_fifo_valid_i,
    output logic                      event_fifo_fulln_o,
    input  logic [EVENT_ID_WIDTH-1:0] event_fifo_data_i, // goes indirectly to core interrupt
    input  logic [31:0]               events_i, // goes directly to core interrupt, should be called irqs
    output logic [1:0]                hwpe_events_o,

    output logic                      supervisor_mode_o
);

    localparam USE_IBEX   = CORE_TYPE == 1 || CORE_TYPE == 2;
    localparam IBEX_RV32M = CORE_TYPE == 1;
    localparam IBEX_RV32E = CORE_TYPE == 2;

    // Interrupt signals
    logic        core_irq_req         ;
    logic        core_irq_sec         ;
    logic [4:0]  core_irq_id          ;
    logic [4:0]  core_irq_ack_id      ;
    logic [4:0]  core_irq_ack_id_RI5CY; // RI5CY irq_id_o
    logic        core_irq_ack         ;
    logic [14:0] core_irq_fast        ;
    logic [31:0] core_irq_fastx       ;
    logic [3:0]  irq_ack_id           ; // ibex irq_id_o

    // Boot address, core id, cluster id, fethc enable and core_status
    logic [31:0] boot_addr        ;
    logic        fetch_en_int     ;
    logic        core_busy_int    ;
    logic        perf_counters_int;
    logic [31:0] hart_id;

    //EU signals
    logic core_clock_en;
    logic fetch_en_eu  ;

    //Core Instr Bus
    logic [31:0] core_instr_addr, core_instr_rdata;
    logic        core_instr_req, core_instr_gnt, core_instr_rvalid, core_instr_err;

    //Core Data Bus
    logic [31:0] core_data_addr, core_data_rdata, core_data_wdata;
    logic        core_data_req, core_data_gnt, core_data_rvalid, core_data_err;
    logic        core_data_we  ;
    logic [ 3:0]  core_data_be ;
    logic is_scm_instr_req, is_scm_data_req;

    assign perf_counters_int = 1'b0;
    assign fetch_en_int      = fetch_en_eu & fetch_en_i;

    assign hart_id = {21'b0, CLUSTER_ID[5:0], 1'b0, CORE_ID[3:0]};

    XBAR_TCDM_BUS core_data_bus ();
    XBAR_TCDM_BUS core_instr_bus ();

    //********************************************************
    //************ CORE DEMUX (TCDM vs L2) *******************
    //********************************************************
`ifdef QUENTIN_SCM
    assign is_scm_instr_req = (core_instr_addr < `SOC_L2_PRI_CH0_SCM_END_ADDR) && (core_instr_addr >= `SOC_L2_PRI_CH0_SCM_START_ADDR) || (core_instr_addr < `ALIAS_SOC_L2_PRI_CH0_SCM_END_ADDR) && (core_instr_addr >= `ALIAS_SOC_L2_PRI_CH0_SCM_START_ADDR);

    fc_demux fc_demux_instr_i (
        .clk          ( clk_i               ),
        .rst_n        ( rst_ni              ),
        .port_sel_i   ( is_scm_instr_req    ),
        .slave_port   ( core_instr_bus      ),
        .master_port0 ( l2_instr_master     ),
        .master_port1 ( scm_l2_instr_master )
    );

    assign core_instr_bus.req   = core_instr_req;
    assign core_instr_bus.add   = core_instr_addr;
    assign core_instr_bus.wen   = ~1'b0;
    assign core_instr_bus.wdata = '0;
    assign core_instr_bus.be    = 4'b1111;
    assign core_instr_gnt       = core_instr_bus.gnt;
    assign core_instr_rvalid    = core_instr_bus.r_valid;
    assign core_instr_rdata     = core_instr_bus.r_rdata;
    assign core_instr_err       = 1'b0;

    assign is_scm_data_req = (core_data_addr < `SOC_L2_PRI_CH0_SCM_END_ADDR) && (core_data_addr >= `SOC_L2_PRI_CH0_SCM_START_ADDR) || (core_data_addr < `ALIAS_SOC_L2_PRI_CH0_SCM_END_ADDR) && (core_data_addr >= `ALIAS_SOC_L2_PRI_CH0_SCM_START_ADDR);

    fc_demux fc_demux_data_i (
        .clk          ( clk_i              ),
        .rst_n        ( rst_ni             ),
        .port_sel_i   ( is_scm_data_req    ),
        .slave_port   ( core_data_bus      ),
        .master_port0 ( l2_data_master     ),
        .master_port1 ( scm_l2_data_master )
    );

    assign core_data_bus.req   = core_data_req;
    assign core_data_bus.add   = core_data_addr;
    assign core_data_bus.wen   = ~core_data_we;
    assign core_data_bus.wdata = core_data_wdata;
    assign core_data_bus.be    = core_data_be;
    assign core_data_gnt       = core_data_bus.gnt;
    assign core_data_rvalid    = core_data_bus.r_valid;
    assign core_data_rdata     = core_data_bus.r_rdata;
    assign core_data_err       = 1'b0;
`else

    assign l2_data_master.req    = core_data_req;
    assign l2_data_master.add    = core_data_addr;
    assign l2_data_master.wen    = ~core_data_we;
    assign l2_data_master.wdata  = core_data_wdata;
    assign l2_data_master.be     = core_data_be;
    assign core_data_gnt         = l2_data_master.gnt;
    assign core_data_rvalid      = l2_data_master.r_valid;
    assign core_data_rdata       = l2_data_master.r_rdata;
    assign core_data_err         = l2_data_master.r_opc;


    assign l2_instr_master.req   = core_instr_req;
    assign l2_instr_master.add   = core_instr_addr;
    assign l2_instr_master.wen   = 1'b1;
    assign l2_instr_master.wdata = '0;
    assign l2_instr_master.be    = 4'b1111;
    assign core_instr_gnt        = l2_instr_master.gnt;
    assign core_instr_rvalid     = l2_instr_master.r_valid;
    assign core_instr_rdata      = l2_instr_master.r_rdata;
    assign core_instr_err        = l2_instr_master.r_opc;


`endif

    //********************************************************
    //************ RISCV CORE ********************************
    //********************************************************
    generate
    if ( USE_IBEX == 0) begin: FC_CORE
    assign boot_addr = boot_addr_i;
    riscv_core #(
        .N_EXT_PERF_COUNTERS ( N_EXT_PERF_COUNTERS ),
        .PULP_SECURE         ( 1                   ),
        .PULP_CLUSTER        ( 0                   ),
        .FPU                 ( USE_FPU             ),
        .FP_DIVSQRT          ( USE_FPU             ),
        .SHARED_FP           ( 0                   ),
        .SHARED_FP_DIVSQRT   ( 2                   )
    ) lFC_CORE (
        .clk_i                 ( clk_i             ),
        .rst_ni                ( rst_ni            ),
        .clock_en_i            ( core_clock_en     ),
        .test_en_i             ( test_en_i         ),
        .boot_addr_i           ( boot_addr         ),
        .core_id_i             ( CORE_ID           ),
        .cluster_id_i          ( CLUSTER_ID        ),

        // Instruction Memory Interface:  Interface to Instruction Logaritmic interconnect: Req->grant handshake
        .instr_addr_o          ( core_instr_addr   ),
        .instr_req_o           ( core_instr_req    ),
        .instr_rdata_i         ( core_instr_rdata  ),
        .instr_gnt_i           ( core_instr_gnt    ),
        .instr_rvalid_i        ( core_instr_rvalid ),

        // Data memory interface:
        .data_addr_o           ( core_data_addr    ),
        .data_req_o            ( core_data_req     ),
        .data_be_o             ( core_data_be      ),
        .data_rdata_i          ( core_data_rdata   ),
        .data_we_o             ( core_data_we      ),
        .data_gnt_i            ( core_data_gnt     ),
        .data_wdata_o          ( core_data_wdata   ),
        .data_rvalid_i         ( core_data_rvalid  ),

        // apu-interconnect
        // handshake signals
        .apu_master_req_o      (                   ),
        .apu_master_ready_o    (                   ),
        .apu_master_gnt_i      ( 1'b1              ),
        // request channel
        .apu_master_operands_o (                   ),
        .apu_master_op_o       (                   ),
        .apu_master_type_o     (                   ),
        .apu_master_flags_o    (                   ),
        // response channel
        .apu_master_valid_i    ( '0                ),
        .apu_master_result_i   ( '0                ),
        .apu_master_flags_i    ( '0                ),

        // TODO
        .irq_software_i        ( 1'b0                  ),
        .irq_timer_i           ( 1'b0                  ),
        .irq_external_i        ( 1'b0                  ),
        .irq_fast_i            ( core_irq_fast         ),
        .irq_nmi_i             ( 1'b0                  ),
        .irq_fastx_i           ( core_irq_fastx        ),
        .irq_ack_o             ( core_irq_ack          ),
        .irq_id_o              ( core_irq_ack_id_RI5CY ),
        .irq_sec_i             ( 1'b0                  ),
        .sec_lvl_o             (                       ),

        .debug_req_i           ( debug_req_i       ),

        .fetch_enable_i        ( fetch_en_int      ),
        .core_busy_o           (                   ),
        .ext_perf_counters_i   ( perf_counters_int ),
        .fregfile_disable_i    ( 1'b0              ) // try me!
    );
    end else begin: FC_CORE
    assign boot_addr = boot_addr_i & 32'hFFFFFF00; // RI5CY expects 0x80 offset, Ibex expects 0x00 offset (adds reset offset 0x80 internally)
`ifdef VERILATOR
    ibex_core #(
`elsif TRACE_EXECUTION
    ibex_core_tracing #(
`else
    ibex_core #(
`endif
        .PMPEnable           ( 0            ),
        .MHPMCounterNum      ( 8            ),
        .MHPMCounterWidth    ( 40           ),
        .RV32E               ( IBEX_RV32E   ),
        .RV32M               ( IBEX_RV32M   ),
        .DmHaltAddr          ( 32'h1A110800 ),
        .DmExceptionAddr     ( 32'h1A110808 )
    ) lFC_CORE (
        .clk_i                 ( clk_i             ),
        .rst_ni                ( rst_ni            ),

        .test_en_i             ( test_en_i         ),

        .hart_id_i             ( hart_id           ),
        .boot_addr_i           ( boot_addr         ),

        // Instruction Memory Interface:  Interface to Instruction Logaritmic interconnect: Req->grant handshake
        .instr_addr_o          ( core_instr_addr   ),
        .instr_req_o           ( core_instr_req    ),
        .instr_rdata_i         ( core_instr_rdata  ),
        .instr_gnt_i           ( core_instr_gnt    ),
        .instr_rvalid_i        ( core_instr_rvalid ),
        .instr_err_i           ( core_instr_err    ),

        // Data memory interface:
        .data_addr_o           ( core_data_addr    ),
        .data_req_o            ( core_data_req     ),
        .data_be_o             ( core_data_be      ),
        .data_rdata_i          ( core_data_rdata   ),
        .data_we_o             ( core_data_we      ),
        .data_gnt_i            ( core_data_gnt     ),
        .data_wdata_o          ( core_data_wdata   ),
        .data_rvalid_i         ( core_data_rvalid  ),
        .data_err_i            ( core_data_err     ),

        .irq_software_i        ( 1'b0              ),
        .irq_timer_i           ( 1'b0              ),
        .irq_external_i        ( 1'b0              ),
        .irq_fast_i            ( core_irq_fast     ),
        .irq_nm_i              ( 1'b0              ),

        .irq_ack_o             ( core_irq_ack      ),
        .irq_ack_id_o          ( irq_ack_id        ),

        .debug_req_i           ( debug_req_i       ),

        .fetch_enable_i        ( fetch_en_int      ),
        .core_sleep_o          (                   )
    );
    end
    endgenerate

    assign supervisor_mode_o = 1'b1;

    generate
    // Remap ack ID for SoC Event FIFO
    // needed for IBEX only since RI5CY does not use hardware ack
    // for std interrupts and theres no need for remap in fastx
    if ( USE_IBEX == 1) begin
        always_comb begin : gen_core_irq_ack_id
            if (irq_ack_id == 10) begin
                core_irq_ack_id = 26;
            end else begin
                core_irq_ack_id = {1'b0, irq_ack_id};
            end
        end

        // Convert ID to std interrupt lines
        always_comb begin : gen_core_irq_fast
            core_irq_fast = '0;
            if (core_irq_req && (core_irq_id == 26)) begin
                // remap SoC Event FIFO
                core_irq_fast[10] = 1'b1;
            end else if (core_irq_req && (core_irq_id < 15)) begin
                core_irq_fast[core_irq_id] = 1'b1;
            end
        end
    end
    // Convert ID to fastx interrupt lines (RI5CY only)
    if ( USE_IBEX == 0) begin
        always_comb begin : gen_core_irq_fast_std
            core_irq_fast = '0;
            if (core_irq_req) begin
                case (core_irq_id)
                    // std irq events                 // FC EVENTS
                    5'd10: core_irq_fast[0] =  1'b1; // s_timer_lo_event
                    5'd11: core_irq_fast[1] =  1'b1; // s_timer_hi_event
                    5'd14: core_irq_fast[2] =  1'b1; // s_ref_rise/fall_event
                    5'd15: core_irq_fast[3] =  1'b1; // s_gpio_event
                    5'd17: core_irq_fast[4] =  1'b1; // s_adv_timer_events[0]
                    5'd18: core_irq_fast[5] =  1'b1; // s_adv_timer_events[1]
                    5'd19: core_irq_fast[6] =  1'b1; // s_adv_timer_events[2]
                    5'd20: core_irq_fast[7] =  1'b1; // s_adv_timer_events[4]
                    5'd26: core_irq_fast[10] = 1'b1; // SoC FIFO event
                    5'd29: core_irq_fast[11] = 1'b1; // s_fc_err_events
                    5'd30: core_irq_fast[12] = 1'b1; // s_fc_hp_events[0]
                    5'd31: core_irq_fast[13] = 1'b1; // s_fc_hp_events[1]

                    default : core_irq_fast = 15'b0;
                endcase
            end
        end

        always_comb begin : gen_core_irq_fastx
            core_irq_fastx = '0;
            if (core_irq_req) begin
                case (core_irq_id)
                    // std irq events                 // FC EVENTS
                    5'd10: core_irq_fastx[16] = 1'b1; // s_timer_lo_event
                    5'd11: core_irq_fastx[17] = 1'b1; // s_timer_hi_event
                    5'd14: core_irq_fastx[18] = 1'b1; // s_ref_rise/fall_event
                    5'd15: core_irq_fastx[19] = 1'b1; // s_gpio_event
                    5'd17: core_irq_fastx[20] = 1'b1; // s_adv_timer_events[0]
                    5'd18: core_irq_fastx[21] = 1'b1; // s_adv_timer_events[1]
                    5'd19: core_irq_fastx[22] = 1'b1; // s_adv_timer_events[2]
                    5'd20: core_irq_fastx[23] = 1'b1; // s_adv_timer_events[4]
                    5'd26: core_irq_fastx[26] = 1'b1; // SoC FIFO event
                    5'd29: core_irq_fastx[27] = 1'b1; // s_fc_err_events
                    5'd30: core_irq_fastx[28] = 1'b1; // s_fc_hp_events[0]
                    5'd31: core_irq_fastx[29] = 1'b1; // s_fc_hp_events[1]
                    // irq x specific events
                    5'd0:  core_irq_fastx[0] = 1'b1; // software event 0
                    5'd1:  core_irq_fastx[1] = 1'b1; // software event 1
                    5'd2:  core_irq_fastx[2] = 1'b1; // software event 2
                    5'd3:  core_irq_fastx[3] = 1'b1; // software event 3
                    5'd4:  core_irq_fastx[4] = 1'b1; // software event 4
                    5'd5:  core_irq_fastx[5] = 1'b1; // software event 5
                    5'd6:  core_irq_fastx[6] = 1'b1; // software event 6
                    5'd7:  core_irq_fastx[7] = 1'b1; // software event 7
                    5'd8:  core_irq_fastx[8] = 1'b1; // dma_pe_evt
                    5'd9:  core_irq_fastx[9] = 1'b1; // dma_pe_irq
                    5'd12: core_irq_fastx[12] = 1'b1; // pf_evt
                    default : core_irq_fastx = 32'b0;
                endcase
            end
        end

        always_comb begin : gen_core_irq_ack_id
            case (core_irq_ack_id_RI5CY)
                5'd16: core_irq_ack_id = 10; // irq fast 0  / fastx 16
                5'd17: core_irq_ack_id = 11; // irq fast 1  / fastx 17
                5'd18: core_irq_ack_id = 14; // irq fast 2  / fastx 18
                5'd19: core_irq_ack_id = 15; // irq fast 3  / fastx 19
                5'd20: core_irq_ack_id = 17; // irq fast 4  / fastx 20
                5'd21: core_irq_ack_id = 18; // irq fast 5  / fastx 21
                5'd22: core_irq_ack_id = 19; // irq fast 6  / fastx 22
                5'd23: core_irq_ack_id = 20; // irq fast 7  / fastx 23
                5'd26: core_irq_ack_id = 26; // irq fast 10 / fastx 26
                5'd27: core_irq_ack_id = 29; // irq fast 11 / fastx 27
                5'd28: core_irq_ack_id = 30; // irq fast 12 / fastx 28
                5'd29: core_irq_ack_id = 31; // irq fast 13 / fastx 29
                // irq x specific events
                5'd0:  core_irq_ack_id = 0; // core_irq_fastx 0 
                5'd1:  core_irq_ack_id = 1; // core_irq_fastx 1 
                5'd2:  core_irq_ack_id = 2; // core_irq_fastx 2 
                5'd3:  core_irq_ack_id = 3; // core_irq_fastx 3 
                5'd4:  core_irq_ack_id = 4; // core_irq_fastx 4 
                5'd5:  core_irq_ack_id = 5; // core_irq_fastx 5 
                5'd6:  core_irq_ack_id = 6; // core_irq_fastx 6 
                5'd7:  core_irq_ack_id = 7; // core_irq_fastx 7 
                5'd8:  core_irq_ack_id = 8; // core_irq_fastx 8 
                5'd9:  core_irq_ack_id = 9; // core_irq_fastx 9 
                5'd12: core_irq_ack_id = 12; // core_irq_fastx 12
                default : core_irq_ack_id = 0;
            endcase
        end
    end

    endgenerate

    apb_interrupt_cntrl #(.PER_ID_WIDTH(PER_ID_WIDTH)) fc_eu_i (
        .clk_i              ( clk_i              ),
        .rst_ni             ( rst_ni             ),
        .test_mode_i        ( test_en_i          ),
        .events_i           ( events_i           ),
        .event_fifo_valid_i ( event_fifo_valid_i ),
        .event_fifo_fulln_o ( event_fifo_fulln_o ),
        .event_fifo_data_i  ( event_fifo_data_i  ),
        .core_secure_mode_i ( 1'b0               ),
        .core_irq_id_o      ( core_irq_id        ),
        .core_irq_req_o     ( core_irq_req       ),
        .core_irq_ack_i     ( core_irq_ack       ),
        .core_irq_id_i      ( core_irq_ack_id    ),
        .core_irq_sec_o     ( /* SECURE IRQ */   ),
        .core_clock_en_o    ( core_clock_en      ),
        .fetch_en_o         ( fetch_en_eu        ),
        .apb_slave          ( apb_slave_eu       )
    );


    generate
    if(USE_HWPE) begin : fc_hwpe_gen
        fc_hwpe #(
            .N_MASTER_PORT ( NB_HWPE_PORTS ),
            .ID_WIDTH      ( 2             )
        ) i_fc_hwpe (
            .clk_i             ( clk_i          ),
            .rst_ni            ( rst_ni         ),
            .test_mode_i       ( test_en_i      ),
            .hwacc_xbar_master ( l2_hwpe_master ),
            .hwacc_cfg_slave   ( apb_slave_hwpe ),
            .evt_o             ( hwpe_events_o  ),
            .busy_o            (                )
        );
    end
    else begin : no_fc_hwpe_gen
        assign hwpe_events_o = '0;
        assign apb_slave_hwpe.prdata  = '0;
        assign apb_slave_hwpe.pready  = '0;
        assign apb_slave_hwpe.pslverr = '0;
        for(genvar ii=0; ii<NB_HWPE_PORTS; ii++) begin
            assign l2_hwpe_master[ii].req   = '0;
            assign l2_hwpe_master[ii].wen   = '0;
            assign l2_hwpe_master[ii].wdata = '0;
            assign l2_hwpe_master[ii].be    = '0;
            assign l2_hwpe_master[ii].add   = '0;
        end
    end
    endgenerate

endmodule

