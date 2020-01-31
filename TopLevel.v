// Generator : SpinalHDL v1.3.5    git head : f0505d24810c8661a24530409359554b7cfa271a
// Date      : 31/01/2020, 21:02:34
// Component : TopLevel


`define CPUModes_defaultEncoding_type [1:0]
`define CPUModes_defaultEncoding_Load 2'b00
`define CPUModes_defaultEncoding_Reset 2'b01
`define CPUModes_defaultEncoding_Pause 2'b10
`define CPUModes_defaultEncoding_Run 2'b11

`define RegSelectModes_defaultEncoding_type [2:0]
`define RegSelectModes_defaultEncoding_PSel 3'b000
`define RegSelectModes_defaultEncoding_NSel 3'b001
`define RegSelectModes_defaultEncoding_XSel 3'b010
`define RegSelectModes_defaultEncoding_DMA0 3'b011
`define RegSelectModes_defaultEncoding_Stack2 3'b100

`define RegOperationModes_defaultEncoding_type [2:0]
`define RegOperationModes_defaultEncoding_None 3'b000
`define RegOperationModes_defaultEncoding_Inc 3'b001
`define RegOperationModes_defaultEncoding_Dec 3'b010
`define RegOperationModes_defaultEncoding_LoadUpper 3'b011
`define RegOperationModes_defaultEncoding_LoadLower 3'b100
`define RegOperationModes_defaultEncoding_UpperOnBus 3'b101
`define RegOperationModes_defaultEncoding_LowerOnBus 3'b110

`define ExecuteModes_defaultEncoding_type [3:0]
`define ExecuteModes_defaultEncoding_None 4'b0000
`define ExecuteModes_defaultEncoding_Load 4'b0001
`define ExecuteModes_defaultEncoding_LoadDec 4'b0010
`define ExecuteModes_defaultEncoding_LoadNoInc 4'b0011
`define ExecuteModes_defaultEncoding_Write 4'b0100
`define ExecuteModes_defaultEncoding_WriteDec 4'b0101
`define ExecuteModes_defaultEncoding_WriteNoInc 4'b0110
`define ExecuteModes_defaultEncoding_LongLoad 4'b0111
`define ExecuteModes_defaultEncoding_LongContinue 4'b1000
`define ExecuteModes_defaultEncoding_DMA_In 4'b1001
`define ExecuteModes_defaultEncoding_DMA_Out 4'b1010

`define BusControlModes_defaultEncoding_type [2:0]
`define BusControlModes_defaultEncoding_DataIn 3'b000
`define BusControlModes_defaultEncoding_DReg 3'b001
`define BusControlModes_defaultEncoding_TReg 3'b010
`define BusControlModes_defaultEncoding_PXReg 3'b011
`define BusControlModes_defaultEncoding_RLower 3'b100
`define BusControlModes_defaultEncoding_RUpper 3'b101

`define DRegControlModes_defaultEncoding_type [3:0]
`define DRegControlModes_defaultEncoding_None 4'b0000
`define DRegControlModes_defaultEncoding_BusIn 4'b0001
`define DRegControlModes_defaultEncoding_ALU_OR 4'b0010
`define DRegControlModes_defaultEncoding_ALU_XOR 4'b0011
`define DRegControlModes_defaultEncoding_ALU_AND 4'b0100
`define DRegControlModes_defaultEncoding_ALU_RSH 4'b0101
`define DRegControlModes_defaultEncoding_ALU_LSH 4'b0110
`define DRegControlModes_defaultEncoding_ALU_RSHR 4'b0111
`define DRegControlModes_defaultEncoding_ALU_LSHR 4'b1000
`define DRegControlModes_defaultEncoding_ALU_Add 4'b1001
`define DRegControlModes_defaultEncoding_ALU_AddCarry 4'b1010
`define DRegControlModes_defaultEncoding_ALU_SubD 4'b1011
`define DRegControlModes_defaultEncoding_ALU_SubDBorrow 4'b1100
`define DRegControlModes_defaultEncoding_ALU_SubM 4'b1101
`define DRegControlModes_defaultEncoding_ALU_SubMBorrow 4'b1110

`define CoreFMS_enumDefinition_defaultEncoding_type [2:0]
`define CoreFMS_enumDefinition_defaultEncoding_boot 3'b000
`define CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset 3'b001
`define CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init 3'b010
`define CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch 3'b011
`define CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute 3'b100
`define CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA 3'b101
`define CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT 3'b110

`define UartParityType_defaultEncoding_type [1:0]
`define UartParityType_defaultEncoding_NONE 2'b00
`define UartParityType_defaultEncoding_EVEN 2'b01
`define UartParityType_defaultEncoding_ODD 2'b10

`define UartStopType_defaultEncoding_type [0:0]
`define UartStopType_defaultEncoding_ONE 1'b0
`define UartStopType_defaultEncoding_TWO 1'b1

`define UartCtrlRxState_defaultEncoding_type [2:0]
`define UartCtrlRxState_defaultEncoding_IDLE 3'b000
`define UartCtrlRxState_defaultEncoding_START 3'b001
`define UartCtrlRxState_defaultEncoding_DATA 3'b010
`define UartCtrlRxState_defaultEncoding_PARITY 3'b011
`define UartCtrlRxState_defaultEncoding_STOP 3'b100

`define UartCtrlTxState_defaultEncoding_type [2:0]
`define UartCtrlTxState_defaultEncoding_IDLE 3'b000
`define UartCtrlTxState_defaultEncoding_START 3'b001
`define UartCtrlTxState_defaultEncoding_DATA 3'b010
`define UartCtrlTxState_defaultEncoding_PARITY 3'b011
`define UartCtrlTxState_defaultEncoding_STOP 3'b100

module BufferCC (
      input   io_initial,
      input   io_dataIn,
      output  io_dataOut,
      input   core8_clk,
      input   core8_reset);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge core8_clk) begin
    if(core8_reset) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module UartCtrlRx (
      input  [2:0] io_configFrame_dataLength,
      input  `UartStopType_defaultEncoding_type io_configFrame_stop,
      input  `UartParityType_defaultEncoding_type io_configFrame_parity,
      input   io_samplingTick,
      output  io_read_valid,
      output [7:0] io_read_payload,
      input   io_rxd,
      input   core8_clk,
      input   core8_reset);
  wire  _zz_1_;
  wire  bufferCC_2__io_dataOut;
  wire  _zz_2_;
  wire  _zz_3_;
  wire [0:0] _zz_4_;
  wire [2:0] _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire  _zz_8_;
  wire  _zz_9_;
  wire  _zz_10_;
  wire  _zz_11_;
  wire  _zz_12_;
  wire  sampler_synchroniser;
  wire  sampler_samples_0;
  reg  sampler_samples_1;
  reg  sampler_samples_2;
  reg  sampler_samples_3;
  reg  sampler_samples_4;
  reg  sampler_value;
  reg  sampler_tick;
  reg [2:0] bitTimer_counter;
  reg  bitTimer_tick;
  reg [2:0] bitCounter_value;
  reg `UartCtrlRxState_defaultEncoding_type stateMachine_state;
  reg  stateMachine_parity;
  reg [7:0] stateMachine_shifter;
  reg  stateMachine_validReg;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif

  assign _zz_2_ = (sampler_tick && (! sampler_value));
  assign _zz_3_ = (bitCounter_value == io_configFrame_dataLength);
  assign _zz_4_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_5_ = {2'd0, _zz_4_};
  assign _zz_6_ = ((((1'b0 || ((_zz_11_ && sampler_samples_1) && sampler_samples_2)) || (((_zz_12_ && sampler_samples_0) && sampler_samples_1) && sampler_samples_3)) || (((1'b1 && sampler_samples_0) && sampler_samples_2) && sampler_samples_3)) || (((1'b1 && sampler_samples_1) && sampler_samples_2) && sampler_samples_3));
  assign _zz_7_ = (((1'b1 && sampler_samples_0) && sampler_samples_1) && sampler_samples_4);
  assign _zz_8_ = ((1'b1 && sampler_samples_0) && sampler_samples_2);
  assign _zz_9_ = (1'b1 && sampler_samples_1);
  assign _zz_10_ = 1'b1;
  assign _zz_11_ = (1'b1 && sampler_samples_0);
  assign _zz_12_ = 1'b1;
  BufferCC bufferCC_2_ ( 
    .io_initial(_zz_1_),
    .io_dataIn(io_rxd),
    .io_dataOut(bufferCC_2__io_dataOut),
    .core8_clk(core8_clk),
    .core8_reset(core8_reset) 
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlRxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlRxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlRxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlRxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  assign _zz_1_ = 1'b0;
  assign sampler_synchroniser = bufferCC_2__io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @ (*) begin
    bitTimer_tick = 1'b0;
    if(sampler_tick)begin
      if((bitTimer_counter == (3'b000)))begin
        bitTimer_tick = 1'b1;
      end
    end
  end

  assign io_read_valid = stateMachine_validReg;
  assign io_read_payload = stateMachine_shifter;
  always @ (posedge core8_clk) begin
    if(core8_reset) begin
      sampler_samples_1 <= 1'b1;
      sampler_samples_2 <= 1'b1;
      sampler_samples_3 <= 1'b1;
      sampler_samples_4 <= 1'b1;
      sampler_value <= 1'b1;
      sampler_tick <= 1'b0;
      stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
      stateMachine_validReg <= 1'b0;
    end else begin
      if(io_samplingTick)begin
        sampler_samples_1 <= sampler_samples_0;
      end
      if(io_samplingTick)begin
        sampler_samples_2 <= sampler_samples_1;
      end
      if(io_samplingTick)begin
        sampler_samples_3 <= sampler_samples_2;
      end
      if(io_samplingTick)begin
        sampler_samples_4 <= sampler_samples_3;
      end
      sampler_value <= ((((((_zz_6_ || _zz_7_) || (_zz_8_ && sampler_samples_4)) || ((_zz_9_ && sampler_samples_2) && sampler_samples_4)) || (((_zz_10_ && sampler_samples_0) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_1) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_2) && sampler_samples_3) && sampler_samples_4));
      sampler_tick <= io_samplingTick;
      stateMachine_validReg <= 1'b0;
      case(stateMachine_state)
        `UartCtrlRxState_defaultEncoding_IDLE : begin
          if(_zz_2_)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_START;
          end
        end
        `UartCtrlRxState_defaultEncoding_START : begin
          if(bitTimer_tick)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_DATA;
            if((sampler_value == 1'b1))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_DATA : begin
          if(bitTimer_tick)begin
            if(_zz_3_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
                stateMachine_validReg <= 1'b1;
              end else begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_PARITY : begin
          if(bitTimer_tick)begin
            if((stateMachine_parity == sampler_value))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
              stateMachine_validReg <= 1'b1;
            end else begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        default : begin
          if(bitTimer_tick)begin
            if((! sampler_value))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end else begin
              if((bitCounter_value == _zz_5_))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
              end
            end
          end
        end
      endcase
    end
  end

  always @ (posedge core8_clk) begin
    if(sampler_tick)begin
      bitTimer_counter <= (bitTimer_counter - (3'b001));
    end
    if(bitTimer_tick)begin
      bitCounter_value <= (bitCounter_value + (3'b001));
    end
    if(bitTimer_tick)begin
      stateMachine_parity <= (stateMachine_parity ^ sampler_value);
    end
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : begin
        if(_zz_2_)begin
          bitTimer_counter <= (3'b010);
        end
      end
      `UartCtrlRxState_defaultEncoding_START : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
        end
      end
      `UartCtrlRxState_defaultEncoding_DATA : begin
        if(bitTimer_tick)begin
          stateMachine_shifter[bitCounter_value] <= sampler_value;
          if(_zz_3_)begin
            bitCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlRxState_defaultEncoding_PARITY : begin
        if(bitTimer_tick)begin
          bitCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module UartCtrlTx (
      input  [2:0] io_configFrame_dataLength,
      input  `UartStopType_defaultEncoding_type io_configFrame_stop,
      input  `UartParityType_defaultEncoding_type io_configFrame_parity,
      input   io_samplingTick,
      input   io_write_valid,
      output reg  io_write_ready,
      input  [7:0] io_write_payload,
      output  io_txd,
      input   core8_clk,
      input   core8_reset);
  wire  _zz_1_;
  wire [0:0] _zz_2_;
  wire [2:0] _zz_3_;
  wire [0:0] _zz_4_;
  wire [2:0] _zz_5_;
  reg  clockDivider_counter_willIncrement;
  wire  clockDivider_counter_willClear;
  reg [2:0] clockDivider_counter_valueNext;
  reg [2:0] clockDivider_counter_value;
  wire  clockDivider_counter_willOverflowIfInc;
  wire  clockDivider_willOverflow;
  reg [2:0] tickCounter_value;
  reg `UartCtrlTxState_defaultEncoding_type stateMachine_state;
  reg  stateMachine_parity;
  reg  stateMachine_txd;
  reg  stateMachine_txd_regNext;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif

  assign _zz_1_ = (tickCounter_value == io_configFrame_dataLength);
  assign _zz_2_ = clockDivider_counter_willIncrement;
  assign _zz_3_ = {2'd0, _zz_2_};
  assign _zz_4_ = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? (1'b0) : (1'b1));
  assign _zz_5_ = {2'd0, _zz_4_};
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlTxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlTxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlTxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlTxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @ (*) begin
    clockDivider_counter_willIncrement = 1'b0;
    if(io_samplingTick)begin
      clockDivider_counter_willIncrement = 1'b1;
    end
  end

  assign clockDivider_counter_willClear = 1'b0;
  assign clockDivider_counter_willOverflowIfInc = (clockDivider_counter_value == (3'b111));
  assign clockDivider_willOverflow = (clockDivider_counter_willOverflowIfInc && clockDivider_counter_willIncrement);
  always @ (*) begin
    clockDivider_counter_valueNext = (clockDivider_counter_value + _zz_3_);
    if(clockDivider_counter_willClear)begin
      clockDivider_counter_valueNext = (3'b000);
    end
  end

  always @ (*) begin
    stateMachine_txd = 1'b1;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        stateMachine_txd = 1'b0;
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        stateMachine_txd = io_write_payload[tickCounter_value];
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        stateMachine_txd = stateMachine_parity;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_write_ready = 1'b0;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_willOverflow)begin
          if(_zz_1_)begin
            io_write_ready = 1'b1;
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
      end
      default : begin
      end
    endcase
  end

  assign io_txd = stateMachine_txd_regNext;
  always @ (posedge core8_clk) begin
    if(core8_reset) begin
      clockDivider_counter_value <= (3'b000);
      stateMachine_state <= `UartCtrlTxState_defaultEncoding_IDLE;
      stateMachine_txd_regNext <= 1'b1;
    end else begin
      clockDivider_counter_value <= clockDivider_counter_valueNext;
      case(stateMachine_state)
        `UartCtrlTxState_defaultEncoding_IDLE : begin
          if((io_write_valid && clockDivider_willOverflow))begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_START;
          end
        end
        `UartCtrlTxState_defaultEncoding_START : begin
          if(clockDivider_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_DATA;
          end
        end
        `UartCtrlTxState_defaultEncoding_DATA : begin
          if(clockDivider_willOverflow)begin
            if(_zz_1_)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
              end else begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlTxState_defaultEncoding_PARITY : begin
          if(clockDivider_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
          end
        end
        default : begin
          if(clockDivider_willOverflow)begin
            if((tickCounter_value == _zz_5_))begin
              stateMachine_state <= (io_write_valid ? `UartCtrlTxState_defaultEncoding_START : `UartCtrlTxState_defaultEncoding_IDLE);
            end
          end
        end
      endcase
      stateMachine_txd_regNext <= stateMachine_txd;
    end
  end

  always @ (posedge core8_clk) begin
    if(clockDivider_willOverflow)begin
      tickCounter_value <= (tickCounter_value + (3'b001));
    end
    if(clockDivider_willOverflow)begin
      stateMachine_parity <= (stateMachine_parity ^ stateMachine_txd);
    end
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        if(clockDivider_willOverflow)begin
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
          tickCounter_value <= (3'b000);
        end
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_willOverflow)begin
          if(_zz_1_)begin
            tickCounter_value <= (3'b000);
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        if(clockDivider_willOverflow)begin
          tickCounter_value <= (3'b000);
        end
      end
      default : begin
      end
    endcase
  end

endmodule

module BufferCC_1_ (
      input   io_initial,
      input   io_dataIn,
      output  io_dataOut,
      input   core8_clk,
      input   _zz_1_);
  reg  buffers_0;
  reg  buffers_1;
  assign io_dataOut = buffers_1;
  always @ (posedge core8_clk or posedge _zz_1_) begin
    if (_zz_1_) begin
      buffers_0 <= io_initial;
      buffers_1 <= io_initial;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end

endmodule

module CDP1802 (
      input   io_Wait_n,
      input   io_Clear_n,
      input   io_DMA_In_n,
      input   io_DMA_Out_n,
      input   io_Interrupt_n,
      input  [3:0] io_EF_n,
      output  io_Q,
      output [1:0] io_SC,
      output [2:0] io_N,
      output  io_TPA,
      output  io_TPB,
      output  io_MRD,
      output  io_MWR,
      output reg [7:0] io_Addr,
      output [15:0] io_Addr16,
      input  [7:0] io_DataIn,
      output [7:0] io_DataOut,
      input   core8_clk,
      input   core8_reset);
  reg [15:0] _zz_24_;
  wire  _zz_25_;
  wire  _zz_26_;
  wire  _zz_27_;
  wire  _zz_28_;
  wire  _zz_29_;
  wire  _zz_30_;
  wire  _zz_31_;
  wire  _zz_32_;
  wire  _zz_33_;
  wire  _zz_34_;
  wire  _zz_35_;
  wire  _zz_36_;
  wire  _zz_37_;
  wire  _zz_38_;
  wire  _zz_39_;
  wire  _zz_40_;
  wire  _zz_41_;
  wire  _zz_42_;
  wire  _zz_43_;
  wire  _zz_44_;
  wire  _zz_45_;
  wire  _zz_46_;
  wire  _zz_47_;
  wire  _zz_48_;
  wire  _zz_49_;
  wire  _zz_50_;
  wire  _zz_51_;
  wire  _zz_52_;
  wire [0:0] _zz_53_;
  wire [2:0] _zz_54_;
  wire [8:0] _zz_55_;
  wire [8:0] _zz_56_;
  wire [8:0] _zz_57_;
  wire [8:0] _zz_58_;
  wire [8:0] _zz_59_;
  wire [8:0] _zz_60_;
  wire [7:0] _zz_61_;
  wire [7:0] _zz_62_;
  reg [1:0] SC;
  reg  Q;
  reg  TPA;
  reg  TPB;
  reg  MRD;
  reg  MWR;
  reg  StateCounter_willIncrement;
  reg  StateCounter_willClear;
  reg [2:0] StateCounter_valueNext;
  reg [2:0] StateCounter_value;
  wire  StateCounter_willOverflowIfInc;
  wire  StateCounter_willOverflow;
  reg  StartCounting;
  reg `CPUModes_defaultEncoding_type Mode;
  reg `RegSelectModes_defaultEncoding_type RegSelMode;
  reg `RegOperationModes_defaultEncoding_type RegOpMode;
  reg `ExecuteModes_defaultEncoding_type ExeMode;
  reg `BusControlModes_defaultEncoding_type BusControl;
  reg `DRegControlModes_defaultEncoding_type DRegControl;
  reg [15:0] Addr;
  reg [15:0] Addr16;
  reg [7:0] D;
  reg [7:0] Dlast;
  reg [2:0] outN;
  reg [3:0] N;
  reg [3:0] I;
  reg [3:0] P;
  reg [3:0] X;
  reg [7:0] T;
  reg [3:0] EF;
  reg  IE;
  reg  DF;
  reg  DFLast;
  reg [7:0] OP;
  reg  Idle;
  reg  Reset;
  reg  Branch;
  reg  Skip;
  wire [8:0] ALU_Add;
  wire [8:0] ALU_AddCarry;
  wire [8:0] ALU_SubD;
  wire [8:0] ALU_SubM;
  wire [8:0] ALU_SubDB;
  wire [8:0] ALU_SubMB;
  reg [7:0] Bus_1_;
  wire [15:0] A;
  reg [3:0] RSel;
  reg [15:0] R_0;
  reg [15:0] R_1;
  reg [15:0] R_2;
  reg [15:0] R_3;
  reg [15:0] R_4;
  reg [15:0] R_5;
  reg [15:0] R_6;
  reg [15:0] R_7;
  reg [15:0] R_8;
  reg [15:0] R_9;
  reg [15:0] R_10;
  reg [15:0] R_11;
  reg [15:0] R_12;
  reg [15:0] R_13;
  reg [15:0] R_14;
  reg [15:0] R_15;
  wire [15:0] _zz_1_;
  wire [15:0] _zz_2_;
  wire  _zz_3_;
  wire  _zz_4_;
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  wire  _zz_8_;
  wire  _zz_9_;
  wire  _zz_10_;
  wire  _zz_11_;
  wire  _zz_12_;
  wire  _zz_13_;
  wire  _zz_14_;
  wire  _zz_15_;
  wire  _zz_16_;
  wire  _zz_17_;
  wire  _zz_18_;
  wire [15:0] _zz_19_;
  wire [15:0] _zz_20_;
  wire [15:0] _zz_21_;
  wire [15:0] _zz_22_;
  wire  CoreFMS_wantExit;
  reg [3:0] _zz_23_;
  reg `CoreFMS_enumDefinition_defaultEncoding_type CoreFMS_stateReg;
  reg `CoreFMS_enumDefinition_defaultEncoding_type CoreFMS_stateNext;
  `ifndef SYNTHESIS
  reg [39:0] Mode_string;
  reg [47:0] RegSelMode_string;
  reg [79:0] RegOpMode_string;
  reg [95:0] ExeMode_string;
  reg [47:0] BusControl_string;
  reg [111:0] DRegControl_string;
  reg [143:0] CoreFMS_stateReg_string;
  reg [143:0] CoreFMS_stateNext_string;
  `endif

  assign _zz_25_ = (_zz_23_ <= (4'b0001));
  assign _zz_26_ = (Mode == `CPUModes_defaultEncoding_Reset);
  assign _zz_27_ = (! io_DMA_In_n);
  assign _zz_28_ = (! io_DMA_Out_n);
  assign _zz_29_ = ((! io_Interrupt_n) && IE);
  assign _zz_30_ = (Mode == `CPUModes_defaultEncoding_Load);
  assign _zz_31_ = (io_DMA_In_n && io_DMA_Out_n);
  assign _zz_32_ = (Mode == `CPUModes_defaultEncoding_Load);
  assign _zz_33_ = (! io_DMA_In_n);
  assign _zz_34_ = (! io_DMA_Out_n);
  assign _zz_35_ = (StateCounter_value == (3'b111));
  assign _zz_36_ = (N == (4'b0000));
  assign _zz_37_ = (((N == (4'b0000)) || (N == (4'b0001))) || (N == (4'b0010)));
  assign _zz_38_ = (N == (4'b0011));
  assign _zz_39_ = (((N == (4'b0100)) || (N == (4'b0101))) || (N == (4'b0111)));
  assign _zz_40_ = (N == (4'b1000));
  assign _zz_41_ = (N == (4'b1001));
  assign _zz_42_ = (StateCounter_value == (3'b001));
  assign _zz_43_ = (StateCounter_value == (3'b101));
  assign _zz_44_ = ((N == (4'b0000)) || (N == (4'b0001)));
  assign _zz_45_ = (N == (4'b0010));
  assign _zz_46_ = ((N == (4'b0100)) || (N == (4'b1100)));
  assign _zz_47_ = ((N == (4'b0101)) || (N == (4'b1101)));
  assign _zz_48_ = (N == (4'b0110));
  assign _zz_49_ = ((N == (4'b0111)) || (N == (4'b1111)));
  assign _zz_50_ = (N == (4'b1010));
  assign _zz_51_ = (N == (4'b1011));
  assign _zz_52_ = (StateCounter_value == (3'b011));
  assign _zz_53_ = StateCounter_willIncrement;
  assign _zz_54_ = {2'd0, _zz_53_};
  assign _zz_55_ = {1'd0, Bus_1_};
  assign _zz_56_ = {1'd0, D};
  assign _zz_57_ = {1'd0, Bus_1_};
  assign _zz_58_ = {1'd0, D};
  assign _zz_59_ = {1'd0, D};
  assign _zz_60_ = {1'd0, Bus_1_};
  assign _zz_61_ = (D >>> 1);
  assign _zz_62_ = (D <<< 1);
  always @(*) begin
    case(RSel)
      4'b0000 : begin
        _zz_24_ = R_0;
      end
      4'b0001 : begin
        _zz_24_ = R_1;
      end
      4'b0010 : begin
        _zz_24_ = R_2;
      end
      4'b0011 : begin
        _zz_24_ = R_3;
      end
      4'b0100 : begin
        _zz_24_ = R_4;
      end
      4'b0101 : begin
        _zz_24_ = R_5;
      end
      4'b0110 : begin
        _zz_24_ = R_6;
      end
      4'b0111 : begin
        _zz_24_ = R_7;
      end
      4'b1000 : begin
        _zz_24_ = R_8;
      end
      4'b1001 : begin
        _zz_24_ = R_9;
      end
      4'b1010 : begin
        _zz_24_ = R_10;
      end
      4'b1011 : begin
        _zz_24_ = R_11;
      end
      4'b1100 : begin
        _zz_24_ = R_12;
      end
      4'b1101 : begin
        _zz_24_ = R_13;
      end
      4'b1110 : begin
        _zz_24_ = R_14;
      end
      default : begin
        _zz_24_ = R_15;
      end
    endcase
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(Mode)
      `CPUModes_defaultEncoding_Load : Mode_string = "Load ";
      `CPUModes_defaultEncoding_Reset : Mode_string = "Reset";
      `CPUModes_defaultEncoding_Pause : Mode_string = "Pause";
      `CPUModes_defaultEncoding_Run : Mode_string = "Run  ";
      default : Mode_string = "?????";
    endcase
  end
  always @(*) begin
    case(RegSelMode)
      `RegSelectModes_defaultEncoding_PSel : RegSelMode_string = "PSel  ";
      `RegSelectModes_defaultEncoding_NSel : RegSelMode_string = "NSel  ";
      `RegSelectModes_defaultEncoding_XSel : RegSelMode_string = "XSel  ";
      `RegSelectModes_defaultEncoding_DMA0 : RegSelMode_string = "DMA0  ";
      `RegSelectModes_defaultEncoding_Stack2 : RegSelMode_string = "Stack2";
      default : RegSelMode_string = "??????";
    endcase
  end
  always @(*) begin
    case(RegOpMode)
      `RegOperationModes_defaultEncoding_None : RegOpMode_string = "None      ";
      `RegOperationModes_defaultEncoding_Inc : RegOpMode_string = "Inc       ";
      `RegOperationModes_defaultEncoding_Dec : RegOpMode_string = "Dec       ";
      `RegOperationModes_defaultEncoding_LoadUpper : RegOpMode_string = "LoadUpper ";
      `RegOperationModes_defaultEncoding_LoadLower : RegOpMode_string = "LoadLower ";
      `RegOperationModes_defaultEncoding_UpperOnBus : RegOpMode_string = "UpperOnBus";
      `RegOperationModes_defaultEncoding_LowerOnBus : RegOpMode_string = "LowerOnBus";
      default : RegOpMode_string = "??????????";
    endcase
  end
  always @(*) begin
    case(ExeMode)
      `ExecuteModes_defaultEncoding_None : ExeMode_string = "None        ";
      `ExecuteModes_defaultEncoding_Load : ExeMode_string = "Load        ";
      `ExecuteModes_defaultEncoding_LoadDec : ExeMode_string = "LoadDec     ";
      `ExecuteModes_defaultEncoding_LoadNoInc : ExeMode_string = "LoadNoInc   ";
      `ExecuteModes_defaultEncoding_Write : ExeMode_string = "Write       ";
      `ExecuteModes_defaultEncoding_WriteDec : ExeMode_string = "WriteDec    ";
      `ExecuteModes_defaultEncoding_WriteNoInc : ExeMode_string = "WriteNoInc  ";
      `ExecuteModes_defaultEncoding_LongLoad : ExeMode_string = "LongLoad    ";
      `ExecuteModes_defaultEncoding_LongContinue : ExeMode_string = "LongContinue";
      `ExecuteModes_defaultEncoding_DMA_In : ExeMode_string = "DMA_In      ";
      `ExecuteModes_defaultEncoding_DMA_Out : ExeMode_string = "DMA_Out     ";
      default : ExeMode_string = "????????????";
    endcase
  end
  always @(*) begin
    case(BusControl)
      `BusControlModes_defaultEncoding_DataIn : BusControl_string = "DataIn";
      `BusControlModes_defaultEncoding_DReg : BusControl_string = "DReg  ";
      `BusControlModes_defaultEncoding_TReg : BusControl_string = "TReg  ";
      `BusControlModes_defaultEncoding_PXReg : BusControl_string = "PXReg ";
      `BusControlModes_defaultEncoding_RLower : BusControl_string = "RLower";
      `BusControlModes_defaultEncoding_RUpper : BusControl_string = "RUpper";
      default : BusControl_string = "??????";
    endcase
  end
  always @(*) begin
    case(DRegControl)
      `DRegControlModes_defaultEncoding_None : DRegControl_string = "None          ";
      `DRegControlModes_defaultEncoding_BusIn : DRegControl_string = "BusIn         ";
      `DRegControlModes_defaultEncoding_ALU_OR : DRegControl_string = "ALU_OR        ";
      `DRegControlModes_defaultEncoding_ALU_XOR : DRegControl_string = "ALU_XOR       ";
      `DRegControlModes_defaultEncoding_ALU_AND : DRegControl_string = "ALU_AND       ";
      `DRegControlModes_defaultEncoding_ALU_RSH : DRegControl_string = "ALU_RSH       ";
      `DRegControlModes_defaultEncoding_ALU_LSH : DRegControl_string = "ALU_LSH       ";
      `DRegControlModes_defaultEncoding_ALU_RSHR : DRegControl_string = "ALU_RSHR      ";
      `DRegControlModes_defaultEncoding_ALU_LSHR : DRegControl_string = "ALU_LSHR      ";
      `DRegControlModes_defaultEncoding_ALU_Add : DRegControl_string = "ALU_Add       ";
      `DRegControlModes_defaultEncoding_ALU_AddCarry : DRegControl_string = "ALU_AddCarry  ";
      `DRegControlModes_defaultEncoding_ALU_SubD : DRegControl_string = "ALU_SubD      ";
      `DRegControlModes_defaultEncoding_ALU_SubDBorrow : DRegControl_string = "ALU_SubDBorrow";
      `DRegControlModes_defaultEncoding_ALU_SubM : DRegControl_string = "ALU_SubM      ";
      `DRegControlModes_defaultEncoding_ALU_SubMBorrow : DRegControl_string = "ALU_SubMBorrow";
      default : DRegControl_string = "??????????????";
    endcase
  end
  always @(*) begin
    case(CoreFMS_stateReg)
      `CoreFMS_enumDefinition_defaultEncoding_boot : CoreFMS_stateReg_string = "boot              ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset : CoreFMS_stateReg_string = "CoreFMS_S1_Reset  ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init : CoreFMS_stateReg_string = "CoreFMS_S1_Init   ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch : CoreFMS_stateReg_string = "CoreFMS_S0_Fetch  ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute : CoreFMS_stateReg_string = "CoreFMS_S1_Execute";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA : CoreFMS_stateReg_string = "CoreFMS_S2_DMA    ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT : CoreFMS_stateReg_string = "CoreFMS_S3_INT    ";
      default : CoreFMS_stateReg_string = "??????????????????";
    endcase
  end
  always @(*) begin
    case(CoreFMS_stateNext)
      `CoreFMS_enumDefinition_defaultEncoding_boot : CoreFMS_stateNext_string = "boot              ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset : CoreFMS_stateNext_string = "CoreFMS_S1_Reset  ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init : CoreFMS_stateNext_string = "CoreFMS_S1_Init   ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch : CoreFMS_stateNext_string = "CoreFMS_S0_Fetch  ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute : CoreFMS_stateNext_string = "CoreFMS_S1_Execute";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA : CoreFMS_stateNext_string = "CoreFMS_S2_DMA    ";
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT : CoreFMS_stateNext_string = "CoreFMS_S3_INT    ";
      default : CoreFMS_stateNext_string = "??????????????????";
    endcase
  end
  `endif

  always @ (*) begin
    StateCounter_willIncrement = 1'b0;
    if((StartCounting && (Mode != `CPUModes_defaultEncoding_Pause)))begin
      StateCounter_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    StateCounter_willClear = 1'b0;
    case(CoreFMS_stateReg)
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset : begin
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init : begin
        StateCounter_willClear = 1'b1;
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch : begin
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute : begin
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA : begin
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT : begin
      end
      default : begin
      end
    endcase
  end

  assign StateCounter_willOverflowIfInc = (StateCounter_value == (3'b111));
  assign StateCounter_willOverflow = (StateCounter_willOverflowIfInc && StateCounter_willIncrement);
  always @ (*) begin
    StateCounter_valueNext = (StateCounter_value + _zz_54_);
    if(StateCounter_willClear)begin
      StateCounter_valueNext = (3'b000);
    end
  end

  assign _zz_1_ = _zz_24_;
  assign _zz_2_ = ({15'd0,(1'b1)} <<< RSel);
  assign _zz_3_ = _zz_2_[0];
  assign _zz_4_ = _zz_2_[1];
  assign _zz_5_ = _zz_2_[2];
  assign _zz_6_ = _zz_2_[3];
  assign _zz_7_ = _zz_2_[4];
  assign _zz_8_ = _zz_2_[5];
  assign _zz_9_ = _zz_2_[6];
  assign _zz_10_ = _zz_2_[7];
  assign _zz_11_ = _zz_2_[8];
  assign _zz_12_ = _zz_2_[9];
  assign _zz_13_ = _zz_2_[10];
  assign _zz_14_ = _zz_2_[11];
  assign _zz_15_ = _zz_2_[12];
  assign _zz_16_ = _zz_2_[13];
  assign _zz_17_ = _zz_2_[14];
  assign _zz_18_ = _zz_2_[15];
  assign A = _zz_1_;
  assign io_Q = Q;
  assign io_SC = SC;
  assign io_N = outN;
  assign io_TPA = TPA;
  assign io_TPB = TPB;
  assign io_MRD = MRD;
  assign io_MWR = MWR;
  assign io_DataOut = Bus_1_;
  assign io_Addr16 = Addr16;
  always @ (*) begin
    if((RegSelMode == `RegSelectModes_defaultEncoding_NSel))begin
      RSel = N;
    end else begin
      if((RegSelMode == `RegSelectModes_defaultEncoding_XSel))begin
        RSel = X;
      end else begin
        if((RegSelMode == `RegSelectModes_defaultEncoding_Stack2))begin
          RSel = (4'b0010);
        end else begin
          if((RegSelMode == `RegSelectModes_defaultEncoding_DMA0))begin
            RSel = (4'b0000);
          end else begin
            RSel = P;
          end
        end
      end
    end
  end

  assign _zz_19_ = (A + (16'b0000000000000001));
  assign _zz_20_ = (A - (16'b0000000000000001));
  assign _zz_21_ = {Bus_1_,_zz_1_[7 : 0]};
  assign _zz_22_ = {_zz_1_[15 : 8],Bus_1_};
  always @ (*) begin
    if((((3'b001) <= StateCounter_value) && (StateCounter_value <= (3'b010))))begin
      io_Addr = Addr[15 : 8];
    end else begin
      io_Addr = Addr[7 : 0];
    end
  end

  assign ALU_Add = (_zz_55_ + _zz_56_);
  assign ALU_AddCarry = (ALU_Add + {(8'b00000000),DF});
  assign ALU_SubD = (_zz_57_ - _zz_58_);
  assign ALU_SubM = (_zz_59_ - _zz_60_);
  assign ALU_SubDB = (ALU_SubD - {(8'b00000000),(! DF)});
  assign ALU_SubMB = (ALU_SubM - {(8'b00000000),(! DF)});
  always @ (*) begin
    if((BusControl == `BusControlModes_defaultEncoding_DataIn))begin
      Bus_1_ = io_DataIn;
    end else begin
      if((BusControl == `BusControlModes_defaultEncoding_DReg))begin
        Bus_1_ = D;
      end else begin
        if((BusControl == `BusControlModes_defaultEncoding_TReg))begin
          Bus_1_ = T;
        end else begin
          if((BusControl == `BusControlModes_defaultEncoding_PXReg))begin
            Bus_1_ = {X,P};
          end else begin
            if((BusControl == `BusControlModes_defaultEncoding_RLower))begin
              Bus_1_ = A[7 : 0];
            end else begin
              if((BusControl == `BusControlModes_defaultEncoding_RUpper))begin
                Bus_1_ = A[15 : 8];
              end else begin
                Bus_1_ = (8'b00000000);
              end
            end
          end
        end
      end
    end
  end

  assign CoreFMS_wantExit = 1'b0;
  always @ (*) begin
    CoreFMS_stateNext = CoreFMS_stateReg;
    case(CoreFMS_stateReg)
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset : begin
        if((Mode != `CPUModes_defaultEncoding_Reset))begin
          CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init;
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init : begin
        if(_zz_25_)begin
          if((Mode == `CPUModes_defaultEncoding_Load))begin
            CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute;
          end else begin
            CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch;
          end
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch : begin
        if(StateCounter_willOverflow)begin
          CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute;
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute : begin
        if(StateCounter_willOverflow)begin
          if(_zz_26_)begin
            CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset;
          end else begin
            if(_zz_27_)begin
              CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA;
            end else begin
              if(_zz_28_)begin
                CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA;
              end else begin
                if(_zz_29_)begin
                  CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT;
                end else begin
                  if(! _zz_30_) begin
                    if(((! ((ExeMode == `ExecuteModes_defaultEncoding_LongLoad) || (ExeMode == `ExecuteModes_defaultEncoding_LongContinue))) && (! Idle)))begin
                      CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch;
                    end
                  end
                end
              end
            end
          end
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA : begin
        if(StateCounter_willOverflow)begin
          if(_zz_31_)begin
            if(_zz_32_)begin
              CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute;
            end else begin
              CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch;
            end
          end
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT : begin
        if(StateCounter_willOverflow)begin
          if(_zz_33_)begin
            CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA;
          end else begin
            if(_zz_34_)begin
              CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA;
            end else begin
              CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch;
            end
          end
        end
      end
      default : begin
        CoreFMS_stateNext = `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset;
      end
    endcase
  end

  always @ (posedge core8_clk) begin
    if(core8_reset) begin
      Q <= 1'b0;
      TPA <= 1'b0;
      TPB <= 1'b0;
      MRD <= 1'b1;
      MWR <= 1'b1;
      StateCounter_value <= (3'b000);
      StartCounting <= 1'b0;
      Addr <= (16'b0000000000000000);
      Addr16 <= (16'b0000000000000000);
      D <= (8'b00000000);
      outN <= (3'b000);
      T <= (8'b00000000);
      IE <= 1'b1;
      DF <= 1'b0;
      Idle <= 1'b0;
      Reset <= 1'b0;
      Branch <= 1'b0;
      CoreFMS_stateReg <= `CoreFMS_enumDefinition_defaultEncoding_boot;
    end else begin
      StateCounter_value <= StateCounter_valueNext;
      Addr16 <= Addr;
      if(((StateCounter_value == (3'b001)) && (Mode != `CPUModes_defaultEncoding_Reset)))begin
        TPA <= 1'b1;
      end else begin
        TPA <= 1'b0;
      end
      if(((StateCounter_value == (3'b110)) && (Mode != `CPUModes_defaultEncoding_Reset)))begin
        TPB <= 1'b1;
      end else begin
        TPB <= 1'b0;
      end
      if((StateCounter_value == (3'b000)))begin
        Addr <= A;
      end else begin
        if(Reset)begin
          Addr <= (16'b0000000000000000);
        end
      end
      if(((3'b011) <= StateCounter_value))begin
        if((SC == (2'b00)))begin
          MRD <= 1'b0;
        end else begin
          if((((SC == (2'b01)) || (SC == (2'b10))) && (((((((ExeMode == `ExecuteModes_defaultEncoding_Load) || (ExeMode == `ExecuteModes_defaultEncoding_LongLoad)) || (ExeMode == `ExecuteModes_defaultEncoding_LoadDec)) || (ExeMode == `ExecuteModes_defaultEncoding_LoadNoInc)) || (ExeMode == `ExecuteModes_defaultEncoding_LongLoad)) || (ExeMode == `ExecuteModes_defaultEncoding_LongContinue)) || (ExeMode == `ExecuteModes_defaultEncoding_DMA_Out))))begin
            MRD <= 1'b0;
          end
        end
      end else begin
        MRD <= 1'b1;
      end
      if((((3'b101) <= StateCounter_value) && (StateCounter_value < (3'b111))))begin
        if((((SC == (2'b01)) || (SC == (2'b10))) && ((((ExeMode == `ExecuteModes_defaultEncoding_Write) || (ExeMode == `ExecuteModes_defaultEncoding_WriteDec)) || (ExeMode == `ExecuteModes_defaultEncoding_WriteNoInc)) || (ExeMode == `ExecuteModes_defaultEncoding_DMA_In))))begin
          MWR <= 1'b0;
        end
      end else begin
        MWR <= 1'b1;
      end
      if((DRegControl == `DRegControlModes_defaultEncoding_BusIn))begin
        D <= Bus_1_;
      end else begin
        if((DRegControl == `DRegControlModes_defaultEncoding_ALU_OR))begin
          D <= (Bus_1_ | D);
        end else begin
          if((DRegControl == `DRegControlModes_defaultEncoding_ALU_XOR))begin
            D <= (Bus_1_ ^ D);
          end else begin
            if((DRegControl == `DRegControlModes_defaultEncoding_ALU_AND))begin
              D <= (Bus_1_ & D);
            end else begin
              if((DRegControl == `DRegControlModes_defaultEncoding_ALU_RSH))begin
                DF <= Dlast[0];
                D <= (D >>> 1);
              end else begin
                if((DRegControl == `DRegControlModes_defaultEncoding_ALU_RSHR))begin
                  DF <= Dlast[0];
                  D <= (_zz_61_ | {DFLast,(7'b0000000)});
                end else begin
                  if((DRegControl == `DRegControlModes_defaultEncoding_ALU_LSH))begin
                    DF <= Dlast[7];
                    D <= (D <<< 1);
                  end else begin
                    if((DRegControl == `DRegControlModes_defaultEncoding_ALU_LSHR))begin
                      DF <= Dlast[7];
                      D <= (_zz_62_ | {(7'b0000000),DFLast});
                    end else begin
                      if((DRegControl == `DRegControlModes_defaultEncoding_ALU_Add))begin
                        DF <= ALU_Add[8];
                        D <= ALU_Add[7:0];
                      end else begin
                        if((DRegControl == `DRegControlModes_defaultEncoding_ALU_AddCarry))begin
                          DF <= ALU_AddCarry[8];
                          D <= ALU_AddCarry[7:0];
                        end else begin
                          if((DRegControl == `DRegControlModes_defaultEncoding_ALU_SubD))begin
                            DF <= (! ALU_SubD[8]);
                            D <= ALU_SubD[7:0];
                          end else begin
                            if((DRegControl == `DRegControlModes_defaultEncoding_ALU_SubM))begin
                              DF <= (! ALU_SubM[8]);
                              D <= ALU_SubM[7:0];
                            end else begin
                              if((DRegControl == `DRegControlModes_defaultEncoding_ALU_SubDBorrow))begin
                                DF <= (! ALU_SubDB[8]);
                                D <= ALU_SubDB[7:0];
                              end else begin
                                if((DRegControl == `DRegControlModes_defaultEncoding_ALU_SubMBorrow))begin
                                  DF <= (! ALU_SubMB[8]);
                                  D <= ALU_SubMB[7:0];
                                end else begin
                                  if(Reset)begin
                                    DF <= 1'b0;
                                    D <= (8'b00000000);
                                  end
                                end
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
      if(((N == (4'b0000)) || ((I == (4'b1100)) && (N == (4'b0100)))))begin
        Branch <= 1'b1;
      end else begin
        if(((N == (4'b0001)) || ((I == (4'b1100)) && (N == (4'b0101)))))begin
          Branch <= (Q == 1'b1);
        end else begin
          if(((N == (4'b0010)) || ((I == (4'b1100)) && (N == (4'b0110)))))begin
            Branch <= (D == (8'b00000000));
          end else begin
            if(((N == (4'b0011)) || ((I == (4'b1100)) && (N == (4'b0111)))))begin
              Branch <= (DF == 1'b1);
            end else begin
              if(((N == (4'b1001)) || ((I == (4'b1100)) && (N == (4'b1101)))))begin
                Branch <= (Q == 1'b0);
              end else begin
                if(((N == (4'b1010)) || ((I == (4'b1100)) && (N == (4'b1110)))))begin
                  Branch <= (D != (8'b00000000));
                end else begin
                  if(((N == (4'b1011)) || ((I == (4'b1100)) && (N == (4'b1111)))))begin
                    Branch <= (DF == 1'b0);
                  end else begin
                    if(((I == (4'b1100)) && (N == (4'b1100))))begin
                      Branch <= (IE == 1'b0);
                    end else begin
                      if(((I == (4'b0011)) && ((((N == (4'b0100)) || (N == (4'b0101))) || (N == (4'b0110))) || (N == (4'b0111)))))begin
                        Branch <= (EF[N[1 : 0]] == 1'b1);
                      end else begin
                        if(((I == (4'b0011)) && ((((N == (4'b1100)) || (N == (4'b1101))) || (N == (4'b1110))) || (N == (4'b1111)))))begin
                          Branch <= (EF[N[1 : 0]] == 1'b0);
                        end else begin
                          Branch <= 1'b0;
                        end
                      end
                    end
                  end
                end
              end
            end
          end
        end
      end
      CoreFMS_stateReg <= CoreFMS_stateNext;
      case(CoreFMS_stateReg)
        `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset : begin
        end
        `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init : begin
          if(_zz_25_)begin
            StartCounting <= 1'b1;
          end
          Reset <= 1'b1;
          Idle <= 1'b0;
          IE <= 1'b1;
          outN <= (3'b000);
          T <= (8'b00000000);
          Q <= 1'b0;
        end
        `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch : begin
          Reset <= 1'b0;
          if(_zz_35_)begin
            case(I)
              4'b0000 : begin
                if(_zz_36_)begin
                  Idle <= 1'b1;
                end
              end
              4'b0001 : begin
              end
              4'b0010 : begin
              end
              4'b0011 : begin
              end
              4'b0100 : begin
              end
              4'b0101 : begin
              end
              4'b0110 : begin
              end
              4'b0111 : begin
                if(! _zz_37_) begin
                  if(! _zz_38_) begin
                    if(! _zz_39_) begin
                      if(! _zz_40_) begin
                        if(_zz_41_)begin
                          T <= {X,P};
                        end
                      end
                    end
                  end
                end
              end
              4'b1000 : begin
              end
              4'b1001 : begin
              end
              4'b1010 : begin
              end
              4'b1011 : begin
              end
              4'b1100 : begin
              end
              4'b1111 : begin
              end
              default : begin
              end
            endcase
          end
        end
        `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute : begin
          Reset <= 1'b0;
          if(_zz_42_)begin
            if(((I == (4'b0110)) && ((4'b0000) < N)))begin
              outN <= N[2:0];
            end
          end
          if(_zz_43_)begin
            case(I)
              4'b0000 : begin
              end
              4'b0001 : begin
              end
              4'b0010 : begin
              end
              4'b0011 : begin
              end
              4'b0100 : begin
              end
              4'b0110 : begin
              end
              4'b0111 : begin
                if(_zz_44_)begin
                  IE <= (! N[0]);
                end else begin
                  if(! _zz_45_) begin
                    if(! _zz_46_) begin
                      if(! _zz_47_) begin
                        if(! _zz_48_) begin
                          if(! _zz_49_) begin
                            if(_zz_50_)begin
                              Q <= 1'b0;
                            end else begin
                              if(_zz_51_)begin
                                Q <= 1'b1;
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
              4'b1000 : begin
              end
              4'b1001 : begin
              end
              4'b1010 : begin
              end
              4'b1011 : begin
              end
              4'b1100 : begin
              end
              4'b1101 : begin
              end
              4'b1110 : begin
              end
              4'b1111 : begin
              end
              default : begin
              end
            endcase
          end
          if(StateCounter_willOverflow)begin
            outN <= (3'b000);
          end
        end
        `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA : begin
          if(StateCounter_willOverflow)begin
            if(_zz_31_)begin
              if(! _zz_32_) begin
                Idle <= 1'b0;
              end
            end
          end
        end
        `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT : begin
          if((StateCounter_value == (3'b010)))begin
            T <= {X,P};
          end
          if(_zz_52_)begin
            IE <= 1'b0;
          end
          if(StateCounter_willOverflow)begin
            Idle <= 1'b0;
          end
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge core8_clk) begin
    Dlast <= D;
    EF <= (~ io_EF_n);
    DFLast <= DF;
    OP <= {I,N};
    Skip <= (((((((((N == (4'b0100)) || (N == (4'b0101))) || (N == (4'b0110))) || (N == (4'b0111))) || (N == (4'b1000))) || (N == (4'b1100))) || (N == (4'b1101))) || (N == (4'b1110))) || (N == (4'b1111)));
    if(((! io_Clear_n) && (! io_Wait_n)))begin
      Mode <= `CPUModes_defaultEncoding_Load;
    end else begin
      if(((! io_Clear_n) && io_Wait_n))begin
        Mode <= `CPUModes_defaultEncoding_Reset;
      end else begin
        if((io_Clear_n && (! io_Wait_n)))begin
          Mode <= `CPUModes_defaultEncoding_Pause;
        end else begin
          Mode <= `CPUModes_defaultEncoding_Run;
        end
      end
    end
    if((RegOpMode == `RegOperationModes_defaultEncoding_Inc))begin
      if(_zz_3_)begin
        R_0 <= _zz_19_;
      end
      if(_zz_4_)begin
        R_1 <= _zz_19_;
      end
      if(_zz_5_)begin
        R_2 <= _zz_19_;
      end
      if(_zz_6_)begin
        R_3 <= _zz_19_;
      end
      if(_zz_7_)begin
        R_4 <= _zz_19_;
      end
      if(_zz_8_)begin
        R_5 <= _zz_19_;
      end
      if(_zz_9_)begin
        R_6 <= _zz_19_;
      end
      if(_zz_10_)begin
        R_7 <= _zz_19_;
      end
      if(_zz_11_)begin
        R_8 <= _zz_19_;
      end
      if(_zz_12_)begin
        R_9 <= _zz_19_;
      end
      if(_zz_13_)begin
        R_10 <= _zz_19_;
      end
      if(_zz_14_)begin
        R_11 <= _zz_19_;
      end
      if(_zz_15_)begin
        R_12 <= _zz_19_;
      end
      if(_zz_16_)begin
        R_13 <= _zz_19_;
      end
      if(_zz_17_)begin
        R_14 <= _zz_19_;
      end
      if(_zz_18_)begin
        R_15 <= _zz_19_;
      end
    end else begin
      if((RegOpMode == `RegOperationModes_defaultEncoding_Dec))begin
        if(_zz_3_)begin
          R_0 <= _zz_20_;
        end
        if(_zz_4_)begin
          R_1 <= _zz_20_;
        end
        if(_zz_5_)begin
          R_2 <= _zz_20_;
        end
        if(_zz_6_)begin
          R_3 <= _zz_20_;
        end
        if(_zz_7_)begin
          R_4 <= _zz_20_;
        end
        if(_zz_8_)begin
          R_5 <= _zz_20_;
        end
        if(_zz_9_)begin
          R_6 <= _zz_20_;
        end
        if(_zz_10_)begin
          R_7 <= _zz_20_;
        end
        if(_zz_11_)begin
          R_8 <= _zz_20_;
        end
        if(_zz_12_)begin
          R_9 <= _zz_20_;
        end
        if(_zz_13_)begin
          R_10 <= _zz_20_;
        end
        if(_zz_14_)begin
          R_11 <= _zz_20_;
        end
        if(_zz_15_)begin
          R_12 <= _zz_20_;
        end
        if(_zz_16_)begin
          R_13 <= _zz_20_;
        end
        if(_zz_17_)begin
          R_14 <= _zz_20_;
        end
        if(_zz_18_)begin
          R_15 <= _zz_20_;
        end
      end else begin
        if((RegOpMode == `RegOperationModes_defaultEncoding_LoadUpper))begin
          if(_zz_3_)begin
            R_0 <= _zz_21_;
          end
          if(_zz_4_)begin
            R_1 <= _zz_21_;
          end
          if(_zz_5_)begin
            R_2 <= _zz_21_;
          end
          if(_zz_6_)begin
            R_3 <= _zz_21_;
          end
          if(_zz_7_)begin
            R_4 <= _zz_21_;
          end
          if(_zz_8_)begin
            R_5 <= _zz_21_;
          end
          if(_zz_9_)begin
            R_6 <= _zz_21_;
          end
          if(_zz_10_)begin
            R_7 <= _zz_21_;
          end
          if(_zz_11_)begin
            R_8 <= _zz_21_;
          end
          if(_zz_12_)begin
            R_9 <= _zz_21_;
          end
          if(_zz_13_)begin
            R_10 <= _zz_21_;
          end
          if(_zz_14_)begin
            R_11 <= _zz_21_;
          end
          if(_zz_15_)begin
            R_12 <= _zz_21_;
          end
          if(_zz_16_)begin
            R_13 <= _zz_21_;
          end
          if(_zz_17_)begin
            R_14 <= _zz_21_;
          end
          if(_zz_18_)begin
            R_15 <= _zz_21_;
          end
        end else begin
          if((RegOpMode == `RegOperationModes_defaultEncoding_LoadLower))begin
            if(_zz_3_)begin
              R_0 <= _zz_22_;
            end
            if(_zz_4_)begin
              R_1 <= _zz_22_;
            end
            if(_zz_5_)begin
              R_2 <= _zz_22_;
            end
            if(_zz_6_)begin
              R_3 <= _zz_22_;
            end
            if(_zz_7_)begin
              R_4 <= _zz_22_;
            end
            if(_zz_8_)begin
              R_5 <= _zz_22_;
            end
            if(_zz_9_)begin
              R_6 <= _zz_22_;
            end
            if(_zz_10_)begin
              R_7 <= _zz_22_;
            end
            if(_zz_11_)begin
              R_8 <= _zz_22_;
            end
            if(_zz_12_)begin
              R_9 <= _zz_22_;
            end
            if(_zz_13_)begin
              R_10 <= _zz_22_;
            end
            if(_zz_14_)begin
              R_11 <= _zz_22_;
            end
            if(_zz_15_)begin
              R_12 <= _zz_22_;
            end
            if(_zz_16_)begin
              R_13 <= _zz_22_;
            end
            if(_zz_17_)begin
              R_14 <= _zz_22_;
            end
            if(_zz_18_)begin
              R_15 <= _zz_22_;
            end
          end else begin
            if(Reset)begin
              R_0 <= (16'b0000000000000000);
            end
          end
        end
      end
    end
    case(CoreFMS_stateReg)
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Reset : begin
        SC <= (2'b01);
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init : begin
        _zz_23_ <= (_zz_23_ - (4'b0001));
        ExeMode <= `ExecuteModes_defaultEncoding_None;
        RegSelMode <= `RegSelectModes_defaultEncoding_PSel;
        RegOpMode <= `RegOperationModes_defaultEncoding_None;
        DRegControl <= `DRegControlModes_defaultEncoding_None;
        BusControl <= `BusControlModes_defaultEncoding_DataIn;
        P <= (4'b0000);
        X <= (4'b0000);
        I <= (4'b0000);
        N <= (4'b0000);
        SC <= (2'b01);
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S0_Fetch : begin
        SC <= (2'b00);
        if((StateCounter_value == (3'b000)))begin
          ExeMode <= `ExecuteModes_defaultEncoding_None;
          BusControl <= `BusControlModes_defaultEncoding_DataIn;
          RegSelMode <= `RegSelectModes_defaultEncoding_PSel;
        end
        if((StateCounter_value == (3'b001)))begin
          RegOpMode <= `RegOperationModes_defaultEncoding_Inc;
        end
        if((StateCounter_value == (3'b010)))begin
          RegOpMode <= `RegOperationModes_defaultEncoding_None;
        end
        if((StateCounter_value == (3'b110)))begin
          I <= io_DataIn[7 : 4];
          N <= io_DataIn[3 : 0];
        end
        if(_zz_35_)begin
          case(I)
            4'b0000 : begin
              if(_zz_36_)begin
                ExeMode <= `ExecuteModes_defaultEncoding_LoadNoInc;
                RegSelMode <= `RegSelectModes_defaultEncoding_DMA0;
              end else begin
                if(((4'b0001) <= N))begin
                  ExeMode <= `ExecuteModes_defaultEncoding_LoadNoInc;
                  RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
                end
              end
            end
            4'b0001 : begin
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b0010 : begin
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b0011 : begin
              ExeMode <= `ExecuteModes_defaultEncoding_Load;
            end
            4'b0100 : begin
              ExeMode <= `ExecuteModes_defaultEncoding_Load;
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b0101 : begin
              ExeMode <= `ExecuteModes_defaultEncoding_WriteNoInc;
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b0110 : begin
              RegSelMode <= `RegSelectModes_defaultEncoding_XSel;
              if((((4'b0000) < N) && (N <= (4'b0111))))begin
                ExeMode <= `ExecuteModes_defaultEncoding_Load;
              end else begin
                if(((4'b1001) <= N))begin
                  ExeMode <= `ExecuteModes_defaultEncoding_WriteNoInc;
                end
              end
            end
            4'b0111 : begin
              if(_zz_37_)begin
                RegSelMode <= `RegSelectModes_defaultEncoding_XSel;
                ExeMode <= `ExecuteModes_defaultEncoding_Load;
              end else begin
                if(_zz_38_)begin
                  RegSelMode <= `RegSelectModes_defaultEncoding_XSel;
                  ExeMode <= `ExecuteModes_defaultEncoding_WriteDec;
                end else begin
                  if(_zz_39_)begin
                    RegSelMode <= `RegSelectModes_defaultEncoding_XSel;
                    ExeMode <= `ExecuteModes_defaultEncoding_LoadNoInc;
                  end else begin
                    if(_zz_40_)begin
                      RegSelMode <= `RegSelectModes_defaultEncoding_XSel;
                      ExeMode <= `ExecuteModes_defaultEncoding_WriteNoInc;
                    end else begin
                      if(_zz_41_)begin
                        RegSelMode <= `RegSelectModes_defaultEncoding_Stack2;
                        ExeMode <= `ExecuteModes_defaultEncoding_WriteDec;
                      end else begin
                        if((((N == (4'b1100)) || (N == (4'b1101))) || (N == (4'b1111))))begin
                          ExeMode <= `ExecuteModes_defaultEncoding_Load;
                        end
                      end
                    end
                  end
                end
              end
            end
            4'b1000 : begin
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b1001 : begin
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b1010 : begin
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b1011 : begin
              RegSelMode <= `RegSelectModes_defaultEncoding_NSel;
            end
            4'b1100 : begin
              ExeMode <= `ExecuteModes_defaultEncoding_Load;
            end
            4'b1111 : begin
              if(((N <= (4'b0101)) || (N == (4'b0111))))begin
                RegSelMode <= `RegSelectModes_defaultEncoding_XSel;
                ExeMode <= `ExecuteModes_defaultEncoding_LoadNoInc;
              end else begin
                if(((((4'b1000) <= N) && (N <= (4'b1101))) || (N == (4'b1111))))begin
                  ExeMode <= `ExecuteModes_defaultEncoding_Load;
                end
              end
            end
            default : begin
            end
          endcase
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Execute : begin
        SC <= (2'b01);
        if(_zz_42_)begin
          if((((ExeMode == `ExecuteModes_defaultEncoding_Load) || (ExeMode == `ExecuteModes_defaultEncoding_Write)) || (ExeMode == `ExecuteModes_defaultEncoding_LongLoad)))begin
            RegOpMode <= `RegOperationModes_defaultEncoding_Inc;
          end else begin
            if((((ExeMode == `ExecuteModes_defaultEncoding_LoadDec) || (ExeMode == `ExecuteModes_defaultEncoding_WriteDec)) || (ExeMode == `ExecuteModes_defaultEncoding_LongContinue)))begin
              RegOpMode <= `RegOperationModes_defaultEncoding_Dec;
            end
          end
        end
        if((StateCounter_value == (3'b010)))begin
          RegOpMode <= `RegOperationModes_defaultEncoding_None;
        end
        if((StateCounter_value == (3'b100)))begin
          case(I)
            4'b0101 : begin
              BusControl <= `BusControlModes_defaultEncoding_DReg;
            end
            4'b0111 : begin
              if(((N == (4'b1000)) || (N == (4'b1001))))begin
                BusControl <= `BusControlModes_defaultEncoding_TReg;
              end else begin
                if((N == (4'b0011)))begin
                  BusControl <= `BusControlModes_defaultEncoding_DReg;
                end
              end
            end
            default : begin
            end
          endcase
        end
        if(_zz_43_)begin
          case(I)
            4'b0000 : begin
              if((N != (4'b0000)))begin
                DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
              end
            end
            4'b0001 : begin
              RegOpMode <= `RegOperationModes_defaultEncoding_Inc;
            end
            4'b0010 : begin
              RegOpMode <= `RegOperationModes_defaultEncoding_Dec;
            end
            4'b0011 : begin
              if(Branch)begin
                RegOpMode <= `RegOperationModes_defaultEncoding_LoadLower;
              end
            end
            4'b0100 : begin
              DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
            end
            4'b0110 : begin
              if((N == (4'b0000)))begin
                RegOpMode <= `RegOperationModes_defaultEncoding_Inc;
              end else begin
                if(((4'b1001) <= N))begin
                  DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
                end
              end
            end
            4'b0111 : begin
              if(_zz_44_)begin
                X <= Bus_1_[7 : 4];
                P <= Bus_1_[3 : 0];
              end else begin
                if(_zz_45_)begin
                  DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
                end else begin
                  if(_zz_46_)begin
                    DRegControl <= `DRegControlModes_defaultEncoding_ALU_AddCarry;
                  end else begin
                    if(_zz_47_)begin
                      DRegControl <= `DRegControlModes_defaultEncoding_ALU_SubDBorrow;
                    end else begin
                      if(_zz_48_)begin
                        DRegControl <= `DRegControlModes_defaultEncoding_ALU_RSHR;
                      end else begin
                        if(_zz_49_)begin
                          DRegControl <= `DRegControlModes_defaultEncoding_ALU_SubMBorrow;
                        end else begin
                          if(! _zz_50_) begin
                            if(! _zz_51_) begin
                              if((N == (4'b1110)))begin
                                DRegControl <= `DRegControlModes_defaultEncoding_ALU_LSHR;
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
            4'b1000 : begin
              BusControl <= `BusControlModes_defaultEncoding_RLower;
              DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
            end
            4'b1001 : begin
              BusControl <= `BusControlModes_defaultEncoding_RUpper;
              DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
            end
            4'b1010 : begin
              BusControl <= `BusControlModes_defaultEncoding_DReg;
              RegOpMode <= `RegOperationModes_defaultEncoding_LoadLower;
            end
            4'b1011 : begin
              BusControl <= `BusControlModes_defaultEncoding_DReg;
              RegOpMode <= `RegOperationModes_defaultEncoding_LoadUpper;
            end
            4'b1100 : begin
              if((ExeMode == `ExecuteModes_defaultEncoding_Load))begin
                if((Branch && (! Skip)))begin
                  RegOpMode <= `RegOperationModes_defaultEncoding_LoadUpper;
                end
              end else begin
                if(((ExeMode == `ExecuteModes_defaultEncoding_LongLoad) && (! Skip)))begin
                  RegOpMode <= `RegOperationModes_defaultEncoding_LoadLower;
                end
              end
            end
            4'b1101 : begin
              P <= N;
            end
            4'b1110 : begin
              X <= N;
            end
            4'b1111 : begin
              if((N == (4'b0000)))begin
                DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
              end else begin
                if(((N == (4'b0001)) || (N == (4'b1001))))begin
                  DRegControl <= `DRegControlModes_defaultEncoding_ALU_OR;
                end else begin
                  if(((N == (4'b0010)) || (N == (4'b1010))))begin
                    DRegControl <= `DRegControlModes_defaultEncoding_ALU_AND;
                  end else begin
                    if(((N == (4'b0011)) || (N == (4'b1011))))begin
                      DRegControl <= `DRegControlModes_defaultEncoding_ALU_XOR;
                    end else begin
                      if(((N == (4'b0100)) || (N == (4'b1100))))begin
                        DRegControl <= `DRegControlModes_defaultEncoding_ALU_Add;
                      end else begin
                        if(((N == (4'b0101)) || (N == (4'b1101))))begin
                          DRegControl <= `DRegControlModes_defaultEncoding_ALU_SubD;
                        end else begin
                          if((N == (4'b0110)))begin
                            DRegControl <= `DRegControlModes_defaultEncoding_ALU_RSH;
                          end else begin
                            if(((N == (4'b0111)) || (N == (4'b1111))))begin
                              DRegControl <= `DRegControlModes_defaultEncoding_ALU_SubM;
                            end else begin
                              if((N == (4'b1000)))begin
                                DRegControl <= `DRegControlModes_defaultEncoding_BusIn;
                              end else begin
                                if((N == (4'b1110)))begin
                                  DRegControl <= `DRegControlModes_defaultEncoding_ALU_LSH;
                                end
                              end
                            end
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
            default : begin
            end
          endcase
        end
        if((StateCounter_value == (3'b110)))begin
          if(((I == (4'b0111)) && (N == (4'b1001))))begin
            X <= P;
          end
          if(((ExeMode == `ExecuteModes_defaultEncoding_LongLoad) || (ExeMode == `ExecuteModes_defaultEncoding_LongContinue)))begin
            ExeMode <= `ExecuteModes_defaultEncoding_None;
          end else begin
            if(((I == (4'b1100)) && ((RegOpMode == `RegOperationModes_defaultEncoding_LoadUpper) || (Skip && (! Branch)))))begin
              ExeMode <= `ExecuteModes_defaultEncoding_LongLoad;
            end else begin
              if(((I == (4'b1100)) && Branch))begin
                ExeMode <= `ExecuteModes_defaultEncoding_LongContinue;
              end
            end
          end
          if(Idle)begin
            RegSelMode <= `RegSelectModes_defaultEncoding_DMA0;
          end else begin
            RegSelMode <= `RegSelectModes_defaultEncoding_PSel;
            RegOpMode <= `RegOperationModes_defaultEncoding_None;
            DRegControl <= `DRegControlModes_defaultEncoding_None;
          end
        end
        if(StateCounter_willOverflow)begin
          if(! _zz_26_) begin
            if(_zz_27_)begin
              RegSelMode <= `RegSelectModes_defaultEncoding_DMA0;
              ExeMode <= `ExecuteModes_defaultEncoding_DMA_In;
            end else begin
              if(_zz_28_)begin
                RegSelMode <= `RegSelectModes_defaultEncoding_DMA0;
                ExeMode <= `ExecuteModes_defaultEncoding_DMA_Out;
              end else begin
                if(_zz_29_)begin
                  ExeMode <= `ExecuteModes_defaultEncoding_None;
                end else begin
                  if(_zz_30_)begin
                    ExeMode <= `ExecuteModes_defaultEncoding_None;
                  end
                end
              end
            end
          end
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S2_DMA : begin
        SC <= (2'b10);
        if((StateCounter_value == (3'b000)))begin
          BusControl <= `BusControlModes_defaultEncoding_DataIn;
          RegSelMode <= `RegSelectModes_defaultEncoding_DMA0;
        end
        if((StateCounter_value == (3'b001)))begin
          RegOpMode <= `RegOperationModes_defaultEncoding_Inc;
        end
        if((StateCounter_value == (3'b010)))begin
          RegOpMode <= `RegOperationModes_defaultEncoding_None;
        end
        if(StateCounter_willOverflow)begin
          if(_zz_31_)begin
            ExeMode <= `ExecuteModes_defaultEncoding_None;
            if(! _zz_32_) begin
              RegSelMode <= `RegSelectModes_defaultEncoding_PSel;
            end
          end
        end
      end
      `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S3_INT : begin
        SC <= (2'b11);
        if(_zz_52_)begin
          P <= (4'b0001);
          X <= (4'b0010);
        end
        if(StateCounter_willOverflow)begin
          if(_zz_33_)begin
            ExeMode <= `ExecuteModes_defaultEncoding_DMA_In;
          end else begin
            if(_zz_34_)begin
              ExeMode <= `ExecuteModes_defaultEncoding_DMA_Out;
            end else begin
              RegSelMode <= `RegSelectModes_defaultEncoding_PSel;
            end
          end
        end
      end
      default : begin
      end
    endcase
    if(((! (CoreFMS_stateReg == `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init)) && (CoreFMS_stateNext == `CoreFMS_enumDefinition_defaultEncoding_CoreFMS_S1_Init)))begin
      _zz_23_ <= (4'b1001);
    end
  end

endmodule

module Ram (
      input   io_ena,
      input  [0:0] io_wea,
      input  [11:0] io_addra,
      output [7:0] io_douta,
      input  [7:0] io_dina,
      input   core8_clk,
      input   core8_reset);
  reg [7:0] _zz_2_;
  wire [11:0] _zz_3_;
  wire  _zz_4_;
  wire [11:0] _zz_1_;
  reg [7:0] mem [0:4095];
  assign _zz_3_ = io_addra;
  assign _zz_4_ = (io_ena && io_wea[0]);
  always @ (posedge core8_clk) begin
    if(_zz_4_) begin
      mem[_zz_3_] <= io_dina;
    end
  end

  always @ (posedge core8_clk) begin
    if(io_ena) begin
      _zz_2_ <= mem[_zz_1_];
    end
  end

  assign _zz_1_ = io_addra;
  assign io_douta = _zz_2_;
endmodule

module uart_rx6 (
      input   io_buffer_reset,
      input   io_serial_in,
      input   io_en_16_x_baud,
      input   io_buffer_read,
      output  io_buffer_data_present,
      output  io_buffer_half_full,
      output  io_buffer_full,
      output [7:0] io_data_out,
      input   core8_clk,
      input   core8_reset);
  wire [2:0] _zz_1_;
  wire `UartStopType_defaultEncoding_type _zz_2_;
  wire `UartParityType_defaultEncoding_type _zz_3_;
  wire  uartCtrlRx_1__io_read_valid;
  wire [7:0] uartCtrlRx_1__io_read_payload;
  reg [19:0] clockDivider_counter;
  wire  clockDivider_tick;
  UartCtrlRx uartCtrlRx_1_ ( 
    .io_configFrame_dataLength(_zz_1_),
    .io_configFrame_stop(_zz_2_),
    .io_configFrame_parity(_zz_3_),
    .io_samplingTick(clockDivider_tick),
    .io_read_valid(uartCtrlRx_1__io_read_valid),
    .io_read_payload(uartCtrlRx_1__io_read_payload),
    .io_rxd(io_serial_in),
    .core8_clk(core8_clk),
    .core8_reset(core8_reset) 
  );
  assign clockDivider_tick = (clockDivider_counter == (20'b00000000000000000000));
  assign _zz_1_ = (3'b111);
  assign _zz_3_ = `UartParityType_defaultEncoding_NONE;
  assign _zz_2_ = `UartStopType_defaultEncoding_ONE;
  assign io_buffer_data_present = uartCtrlRx_1__io_read_valid;
  assign io_buffer_half_full = 1'b0;
  assign io_buffer_full = 1'b0;
  assign io_data_out = uartCtrlRx_1__io_read_payload;
  always @ (posedge core8_clk) begin
    if(core8_reset) begin
      clockDivider_counter <= (20'b00000000000000000000);
    end else begin
      clockDivider_counter <= (clockDivider_counter - (20'b00000000000000000001));
      if(clockDivider_tick)begin
        clockDivider_counter <= (20'b00000000000000001000);
      end
    end
  end

endmodule

module uart_tx6 (
      input   io_buffer_reset,
      input  [7:0] io_data_in,
      input   io_en_16_x_baud,
      input   io_buffer_write,
      output  io_serial_out,
      output  io_buffer_data_present,
      output  io_buffer_half_full,
      output  io_buffer_full,
      input   core8_clk,
      input   core8_reset);
  wire [2:0] _zz_3_;
  wire `UartStopType_defaultEncoding_type _zz_4_;
  wire `UartParityType_defaultEncoding_type _zz_5_;
  wire  uartCtrlTx_1__io_write_ready;
  wire  uartCtrlTx_1__io_txd;
  reg [19:0] clockDivider_counter;
  wire  clockDivider_tick;
  wire  write_valid;
  wire  write_ready;
  wire [7:0] write_payload;
  wire  write_m2sPipe_valid;
  wire  write_m2sPipe_ready;
  wire [7:0] write_m2sPipe_payload;
  reg  _zz_1_;
  reg [7:0] _zz_2_;
  UartCtrlTx uartCtrlTx_1_ ( 
    .io_configFrame_dataLength(_zz_3_),
    .io_configFrame_stop(_zz_4_),
    .io_configFrame_parity(_zz_5_),
    .io_samplingTick(clockDivider_tick),
    .io_write_valid(write_m2sPipe_valid),
    .io_write_ready(uartCtrlTx_1__io_write_ready),
    .io_write_payload(write_m2sPipe_payload),
    .io_txd(uartCtrlTx_1__io_txd),
    .core8_clk(core8_clk),
    .core8_reset(core8_reset) 
  );
  assign clockDivider_tick = (clockDivider_counter == (20'b00000000000000000000));
  assign _zz_3_ = (3'b111);
  assign _zz_5_ = `UartParityType_defaultEncoding_NONE;
  assign _zz_4_ = `UartStopType_defaultEncoding_ONE;
  assign io_serial_out = uartCtrlTx_1__io_txd;
  assign write_valid = io_buffer_write;
  assign write_payload = io_data_in;
  assign write_ready = ((1'b1 && (! write_m2sPipe_valid)) || write_m2sPipe_ready);
  assign write_m2sPipe_valid = _zz_1_;
  assign write_m2sPipe_payload = _zz_2_;
  assign write_m2sPipe_ready = uartCtrlTx_1__io_write_ready;
  assign io_buffer_data_present = 1'b0;
  assign io_buffer_half_full = 1'b0;
  assign io_buffer_full = 1'b0;
  always @ (posedge core8_clk) begin
    if(core8_reset) begin
      clockDivider_counter <= (20'b00000000000000000000);
      _zz_1_ <= 1'b0;
    end else begin
      clockDivider_counter <= (clockDivider_counter - (20'b00000000000000000001));
      if(clockDivider_tick)begin
        clockDivider_counter <= (20'b00000000000000001000);
      end
      if(write_ready)begin
        _zz_1_ <= write_valid;
      end
    end
  end

  always @ (posedge core8_clk) begin
    if(write_ready)begin
      _zz_2_ <= write_payload;
    end
  end

endmodule

module TopLevel (
      input   clkin,
      input   reset_n,
      input  [11:0] switches,
      output [7:0] LEDs,
      output [9:0] segdis,
      input   avr_tx,
      output  avr_rx);
  wire  _zz_5_;
  wire  _zz_6_;
  wire  _zz_7_;
  reg  _zz_8_;
  wire  _zz_9_;
  wire  _zz_10_;
  wire  _zz_11_;
  wire  _zz_12_;
  reg [3:0] _zz_13_;
  reg [7:0] _zz_14_;
  wire  _zz_15_;
  wire [0:0] _zz_16_;
  wire [11:0] _zz_17_;
  wire  _zz_18_;
  wire  _zz_19_;
  wire  _zz_20_;
  wire  _zz_21_;
  reg [3:0] _zz_22_;
  reg [6:0] _zz_23_;
  wire  clkCtrl_pll_CLK_OUT1;
  wire  clkCtrl_pll_LOCKED;
  wire  bufferCC_2__io_dataOut;
  wire  core8_Cpu_io_Q;
  wire [1:0] core8_Cpu_io_SC;
  wire [2:0] core8_Cpu_io_N;
  wire  core8_Cpu_io_TPA;
  wire  core8_Cpu_io_TPB;
  wire  core8_Cpu_io_MRD;
  wire  core8_Cpu_io_MWR;
  wire [7:0] core8_Cpu_io_Addr;
  wire [15:0] core8_Cpu_io_Addr16;
  wire [7:0] core8_Cpu_io_DataOut;
  wire [7:0] core8_ram4096_io_douta;
  wire  core8_UartRx_io_buffer_data_present;
  wire  core8_UartRx_io_buffer_half_full;
  wire  core8_UartRx_io_buffer_full;
  wire [7:0] core8_UartRx_io_data_out;
  wire  core8_UartTx_io_serial_out;
  wire  core8_UartTx_io_buffer_data_present;
  wire  core8_UartTx_io_buffer_half_full;
  wire  core8_UartTx_io_buffer_full;
  wire  _zz_24_;
  wire [0:0] _zz_25_;
  wire [18:0] _zz_26_;
  wire [11:0] _zz_27_;
  wire [0:0] _zz_28_;
  wire [9:0] _zz_29_;
  wire [0:0] _zz_30_;
  wire [1:0] _zz_31_;
  wire [6:0] _zz_32_;
  wire [6:0] _zz_33_;
  wire [6:0] _zz_34_;
  wire [6:0] _zz_35_;
  wire [6:0] _zz_36_;
  wire [6:0] _zz_37_;
  wire [6:0] _zz_38_;
  wire [6:0] _zz_39_;
  wire [6:0] _zz_40_;
  wire [6:0] _zz_41_;
  wire [6:0] _zz_42_;
  wire [6:0] _zz_43_;
  wire [6:0] _zz_44_;
  wire [6:0] _zz_45_;
  wire [6:0] _zz_46_;
  wire [6:0] _zz_47_;
  wire [0:0] _zz_48_;
  wire [0:0] _zz_49_;
  wire [0:0] _zz_50_;
  wire [1:0] _zz_51_;
  wire [0:0] _zz_52_;
  wire [7:0] _zz_53_;
  wire [3:0] _zz_54_;
  wire [3:0] _zz_55_;
  wire [11:0] _zz_56_;
  wire [11:0] _zz_57_;
  wire [11:0] _zz_58_;
  wire [11:0] _zz_59_;
  wire [11:0] _zz_60_;
  wire [11:0] _zz_61_;
  wire [11:0] _zz_62_;
  wire [11:0] _zz_63_;
  wire [11:0] _zz_64_;
  wire [11:0] _zz_65_;
  wire [11:0] _zz_66_;
  wire [11:0] _zz_67_;
  wire [11:0] _zz_68_;
  wire [11:0] _zz_69_;
  wire [11:0] _zz_70_;
  wire [11:0] _zz_71_;
  wire [3:0] _zz_72_;
  wire  core8_clk;
  wire  core8_reset;
  wire  _zz_1_;
  reg  core8_step;
  reg  core8_stepDMAIn;
  reg [7:0] core8_DMADataIN;
  reg  core8_interrupt;
  wire  core8_interruptLast;
  reg  core8_debounce_timer_state;
  reg  core8_debounce_timer_stateRise;
  wire  core8_debounce_timer_counter_willIncrement;
  reg  core8_debounce_timer_counter_willClear;
  reg [18:0] core8_debounce_timer_counter_valueNext;
  reg [18:0] core8_debounce_timer_counter_value;
  wire  core8_debounce_timer_counter_willOverflowIfInc;
  wire  core8_debounce_timer_counter_willOverflow;
  reg [11:0] core8_debounce_input;
  reg [11:0] core8_debounce_debounce;
  reg [11:0] core8_debounce_value;
  reg  core8_SevenSegment_dp;
  reg [7:0] core8_SevenSegment_segments;
  reg [1:0] core8_SevenSegment_displays;
  reg  core8_SevenSegment_timer_state;
  reg  core8_SevenSegment_timer_stateRise;
  wire  core8_SevenSegment_timer_counter_willIncrement;
  reg  core8_SevenSegment_timer_counter_willClear;
  reg [9:0] core8_SevenSegment_timer_counter_valueNext;
  reg [9:0] core8_SevenSegment_timer_counter_value;
  wire  core8_SevenSegment_timer_counter_willOverflowIfInc;
  wire  core8_SevenSegment_timer_counter_willOverflow;
  reg  core8_SevenSegment_displayCounter_willIncrement;
  wire  core8_SevenSegment_displayCounter_willClear;
  reg [1:0] core8_SevenSegment_displayCounter_valueNext;
  reg [1:0] core8_SevenSegment_displayCounter_value;
  wire  core8_SevenSegment_displayCounter_willOverflowIfInc;
  wire  core8_SevenSegment_displayCounter_willOverflow;
  reg [3:0] core8_SevenSegment_digits_0;
  reg [3:0] core8_SevenSegment_digits_1;
  wire [3:0] core8_SevenSegment_digit;
  wire [6:0] core8_SevenSegment_digit2segments_0;
  wire [6:0] core8_SevenSegment_digit2segments_1;
  wire [6:0] core8_SevenSegment_digit2segments_2;
  wire [6:0] core8_SevenSegment_digit2segments_3;
  wire [6:0] core8_SevenSegment_digit2segments_4;
  wire [6:0] core8_SevenSegment_digit2segments_5;
  wire [6:0] core8_SevenSegment_digit2segments_6;
  wire [6:0] core8_SevenSegment_digit2segments_7;
  wire [6:0] core8_SevenSegment_digit2segments_8;
  wire [6:0] core8_SevenSegment_digit2segments_9;
  wire [6:0] core8_SevenSegment_digit2segments_10;
  wire [6:0] core8_SevenSegment_digit2segments_11;
  wire [6:0] core8_SevenSegment_digit2segments_12;
  wire [6:0] core8_SevenSegment_digit2segments_13;
  wire [6:0] core8_SevenSegment_digit2segments_14;
  wire [6:0] core8_SevenSegment_digit2segments_15;
  wire [7:0] _zz_2_;
  wire [7:0] core8_serialDataRX;
  wire  core8_serialDataPresent;
  reg  core8_buffer_read;
  reg  core8_buffer_read_last;
  wire  core8_serialDataSend;
  wire [7:0] core8_DMADataOut;
  wire  _zz_3_;
  reg  _zz_3__regNext;
  wire  _zz_4_;
  reg  _zz_4__regNext;
  reg  core8_serialDataPresent_regNext;
  assign _zz_24_ = (! core8_SevenSegment_displayCounter_value[0]);
  assign _zz_25_ = core8_debounce_timer_counter_willIncrement;
  assign _zz_26_ = {18'd0, _zz_25_};
  assign _zz_27_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_28_ = core8_SevenSegment_timer_counter_willIncrement;
  assign _zz_29_ = {9'd0, _zz_28_};
  assign _zz_30_ = core8_SevenSegment_displayCounter_willIncrement;
  assign _zz_31_ = {1'd0, _zz_30_};
  assign _zz_32_ = (7'b0111111);
  assign _zz_33_ = (7'b0000110);
  assign _zz_34_ = (7'b1011011);
  assign _zz_35_ = (7'b1001111);
  assign _zz_36_ = (7'b1100110);
  assign _zz_37_ = (7'b1101101);
  assign _zz_38_ = (7'b1111101);
  assign _zz_39_ = (7'b0000111);
  assign _zz_40_ = (7'b1111111);
  assign _zz_41_ = (7'b1101111);
  assign _zz_42_ = (7'b1110111);
  assign _zz_43_ = (7'b1111100);
  assign _zz_44_ = (7'b0111001);
  assign _zz_45_ = (7'b1011110);
  assign _zz_46_ = (7'b1111001);
  assign _zz_47_ = (7'b1110001);
  assign _zz_48_ = (core8_SevenSegment_displayCounter_value >>> 1);
  assign _zz_49_ = ((1'b0) - (1'b1));
  assign _zz_50_ = (core8_SevenSegment_displayCounter_value >>> 1);
  assign _zz_51_ = ({1'd0,(1'b1)} <<< _zz_52_);
  assign _zz_52_ = (core8_SevenSegment_displayCounter_value >>> 1);
  assign _zz_53_ = (_zz_2_ & (8'b00001111));
  assign _zz_54_ = _zz_55_;
  assign _zz_55_ = ((_zz_2_ & (8'b11110000)) >>> 4);
  assign _zz_56_ = (core8_debounce_value & (~ core8_debounce_debounce));
  assign _zz_57_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_58_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_59_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_60_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_61_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_62_ = ((~ core8_debounce_value) & core8_debounce_debounce);
  assign _zz_63_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_64_ = ((~ core8_debounce_value) & core8_debounce_debounce);
  assign _zz_65_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_66_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_67_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_68_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_69_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_70_ = ((~ core8_debounce_value) & core8_debounce_debounce);
  assign _zz_71_ = (core8_debounce_value & core8_debounce_debounce);
  assign _zz_72_ = core8_SevenSegment_digit;
  PLL clkCtrl_pll ( 
    .CLK_IN1(clkin),
    .RESET(_zz_5_),
    .CLK_OUT1(clkCtrl_pll_CLK_OUT1),
    .LOCKED(clkCtrl_pll_LOCKED) 
  );
  BufferCC_1_ bufferCC_2_ ( 
    .io_initial(_zz_6_),
    .io_dataIn(_zz_7_),
    .io_dataOut(bufferCC_2__io_dataOut),
    .core8_clk(core8_clk),
    ._zz_1_(_zz_1_) 
  );
  CDP1802 core8_Cpu ( 
    .io_Wait_n(_zz_8_),
    .io_Clear_n(_zz_9_),
    .io_DMA_In_n(_zz_10_),
    .io_DMA_Out_n(_zz_11_),
    .io_Interrupt_n(_zz_12_),
    .io_EF_n(_zz_13_),
    .io_Q(core8_Cpu_io_Q),
    .io_SC(core8_Cpu_io_SC),
    .io_N(core8_Cpu_io_N),
    .io_TPA(core8_Cpu_io_TPA),
    .io_TPB(core8_Cpu_io_TPB),
    .io_MRD(core8_Cpu_io_MRD),
    .io_MWR(core8_Cpu_io_MWR),
    .io_Addr(core8_Cpu_io_Addr),
    .io_Addr16(core8_Cpu_io_Addr16),
    .io_DataIn(_zz_14_),
    .io_DataOut(core8_Cpu_io_DataOut),
    .core8_clk(core8_clk),
    .core8_reset(core8_reset) 
  );
  Ram core8_ram4096 ( 
    .io_ena(_zz_15_),
    .io_wea(_zz_16_),
    .io_addra(_zz_17_),
    .io_douta(core8_ram4096_io_douta),
    .io_dina(core8_Cpu_io_DataOut),
    .core8_clk(core8_clk),
    .core8_reset(core8_reset) 
  );
  uart_rx6 core8_UartRx ( 
    .io_buffer_reset(_zz_18_),
    .io_serial_in(avr_tx),
    .io_en_16_x_baud(_zz_19_),
    .io_buffer_read(_zz_20_),
    .io_buffer_data_present(core8_UartRx_io_buffer_data_present),
    .io_buffer_half_full(core8_UartRx_io_buffer_half_full),
    .io_buffer_full(core8_UartRx_io_buffer_full),
    .io_data_out(core8_UartRx_io_data_out),
    .core8_clk(core8_clk),
    .core8_reset(core8_reset) 
  );
  uart_tx6 core8_UartTx ( 
    .io_buffer_reset(core8_reset),
    .io_data_in(core8_Cpu_io_DataOut),
    .io_en_16_x_baud(_zz_21_),
    .io_buffer_write(core8_serialDataSend),
    .io_serial_out(core8_UartTx_io_serial_out),
    .io_buffer_data_present(core8_UartTx_io_buffer_data_present),
    .io_buffer_half_full(core8_UartTx_io_buffer_half_full),
    .io_buffer_full(core8_UartTx_io_buffer_full),
    .core8_clk(core8_clk),
    .core8_reset(core8_reset) 
  );
  always @(*) begin
    case(_zz_48_)
      1'b0 : begin
        _zz_22_ = core8_SevenSegment_digits_0;
      end
      default : begin
        _zz_22_ = core8_SevenSegment_digits_1;
      end
    endcase
  end

  always @(*) begin
    case(_zz_72_)
      4'b0000 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_0;
      end
      4'b0001 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_1;
      end
      4'b0010 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_2;
      end
      4'b0011 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_3;
      end
      4'b0100 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_4;
      end
      4'b0101 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_5;
      end
      4'b0110 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_6;
      end
      4'b0111 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_7;
      end
      4'b1000 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_8;
      end
      4'b1001 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_9;
      end
      4'b1010 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_10;
      end
      4'b1011 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_11;
      end
      4'b1100 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_12;
      end
      4'b1101 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_13;
      end
      4'b1110 : begin
        _zz_23_ = core8_SevenSegment_digit2segments_14;
      end
      default : begin
        _zz_23_ = core8_SevenSegment_digit2segments_15;
      end
    endcase
  end

  assign _zz_5_ = (! reset_n);
  assign core8_clk = clkCtrl_pll_CLK_OUT1;
  assign _zz_1_ = ((! reset_n) || (! clkCtrl_pll_LOCKED));
  assign _zz_7_ = 1'b0;
  assign _zz_6_ = 1'b1;
  assign core8_reset = bufferCC_2__io_dataOut;
  assign core8_interruptLast = 1'b0;
  assign _zz_10_ = (! core8_stepDMAIn);
  assign _zz_11_ = 1'b1;
  assign _zz_12_ = (! (core8_interrupt && (! core8_interruptLast)));
  always @ (*) begin
    _zz_13_[2] = 1'b1;
    _zz_13_[1] = core8_UartRx_io_buffer_data_present;
    _zz_13_[0] = core8_UartTx_io_buffer_full;
    _zz_13_[3] = _zz_61_[9];
  end

  always @ (*) begin
    core8_debounce_timer_stateRise = 1'b0;
    if(core8_debounce_timer_counter_willOverflow)begin
      core8_debounce_timer_stateRise = (! core8_debounce_timer_state);
    end
    if(core8_debounce_timer_state)begin
      core8_debounce_timer_stateRise = 1'b0;
    end
  end

  always @ (*) begin
    core8_debounce_timer_counter_willClear = 1'b0;
    if(core8_debounce_timer_state)begin
      core8_debounce_timer_counter_willClear = 1'b1;
    end
  end

  assign core8_debounce_timer_counter_willOverflowIfInc = (core8_debounce_timer_counter_value == (19'b1100001101001111111));
  assign core8_debounce_timer_counter_willOverflow = (core8_debounce_timer_counter_willOverflowIfInc && core8_debounce_timer_counter_willIncrement);
  always @ (*) begin
    if(core8_debounce_timer_counter_willOverflow)begin
      core8_debounce_timer_counter_valueNext = (19'b0000000000000000000);
    end else begin
      core8_debounce_timer_counter_valueNext = (core8_debounce_timer_counter_value + _zz_26_);
    end
    if(core8_debounce_timer_counter_willClear)begin
      core8_debounce_timer_counter_valueNext = (19'b0000000000000000000);
    end
  end

  assign core8_debounce_timer_counter_willIncrement = 1'b1;
  assign _zz_15_ = 1'b1;
  assign _zz_16_ = (~ (core8_Cpu_io_MWR && (! _zz_27_[8])));
  assign _zz_17_ = core8_Cpu_io_Addr16[11 : 0];
  always @ (*) begin
    core8_SevenSegment_timer_stateRise = 1'b0;
    if(core8_SevenSegment_timer_counter_willOverflow)begin
      core8_SevenSegment_timer_stateRise = (! core8_SevenSegment_timer_state);
    end
    if(core8_SevenSegment_timer_state)begin
      core8_SevenSegment_timer_stateRise = 1'b0;
    end
  end

  always @ (*) begin
    core8_SevenSegment_timer_counter_willClear = 1'b0;
    if(core8_SevenSegment_timer_state)begin
      core8_SevenSegment_timer_counter_willClear = 1'b1;
    end
  end

  assign core8_SevenSegment_timer_counter_willOverflowIfInc = (core8_SevenSegment_timer_counter_value == (10'b1100011111));
  assign core8_SevenSegment_timer_counter_willOverflow = (core8_SevenSegment_timer_counter_willOverflowIfInc && core8_SevenSegment_timer_counter_willIncrement);
  always @ (*) begin
    if(core8_SevenSegment_timer_counter_willOverflow)begin
      core8_SevenSegment_timer_counter_valueNext = (10'b0000000000);
    end else begin
      core8_SevenSegment_timer_counter_valueNext = (core8_SevenSegment_timer_counter_value + _zz_29_);
    end
    if(core8_SevenSegment_timer_counter_willClear)begin
      core8_SevenSegment_timer_counter_valueNext = (10'b0000000000);
    end
  end

  assign core8_SevenSegment_timer_counter_willIncrement = 1'b1;
  always @ (*) begin
    core8_SevenSegment_displayCounter_willIncrement = 1'b0;
    if(core8_SevenSegment_timer_state)begin
      core8_SevenSegment_displayCounter_willIncrement = 1'b1;
    end
  end

  assign core8_SevenSegment_displayCounter_willClear = 1'b0;
  assign core8_SevenSegment_displayCounter_willOverflowIfInc = (core8_SevenSegment_displayCounter_value == (2'b11));
  assign core8_SevenSegment_displayCounter_willOverflow = (core8_SevenSegment_displayCounter_willOverflowIfInc && core8_SevenSegment_displayCounter_willIncrement);
  always @ (*) begin
    core8_SevenSegment_displayCounter_valueNext = (core8_SevenSegment_displayCounter_value + _zz_31_);
    if(core8_SevenSegment_displayCounter_willClear)begin
      core8_SevenSegment_displayCounter_valueNext = (2'b00);
    end
  end

  assign core8_SevenSegment_digit2segments_0 = _zz_32_;
  assign core8_SevenSegment_digit2segments_1 = _zz_33_;
  assign core8_SevenSegment_digit2segments_2 = _zz_34_;
  assign core8_SevenSegment_digit2segments_3 = _zz_35_;
  assign core8_SevenSegment_digit2segments_4 = _zz_36_;
  assign core8_SevenSegment_digit2segments_5 = _zz_37_;
  assign core8_SevenSegment_digit2segments_6 = _zz_38_;
  assign core8_SevenSegment_digit2segments_7 = _zz_39_;
  assign core8_SevenSegment_digit2segments_8 = _zz_40_;
  assign core8_SevenSegment_digit2segments_9 = _zz_41_;
  assign core8_SevenSegment_digit2segments_10 = _zz_42_;
  assign core8_SevenSegment_digit2segments_11 = _zz_43_;
  assign core8_SevenSegment_digit2segments_12 = _zz_44_;
  assign core8_SevenSegment_digit2segments_13 = _zz_45_;
  assign core8_SevenSegment_digit2segments_14 = _zz_46_;
  assign core8_SevenSegment_digit2segments_15 = _zz_47_;
  assign core8_SevenSegment_digit = _zz_22_;
  always @ (*) begin
    if(1'b0)begin
      core8_SevenSegment_dp = (_zz_49_ == _zz_50_);
    end else begin
      core8_SevenSegment_dp = 1'b0;
    end
  end

  always @ (*) begin
    if(_zz_24_)begin
      core8_SevenSegment_displays = (~ _zz_51_);
    end else begin
      core8_SevenSegment_displays = (2'b00);
    end
  end

  always @ (*) begin
    if(_zz_24_)begin
      core8_SevenSegment_segments = {core8_SevenSegment_dp,_zz_23_};
    end else begin
      core8_SevenSegment_segments = (8'b00000000);
    end
  end

  assign segdis = {core8_SevenSegment_displays,core8_SevenSegment_segments};
  assign _zz_2_ = core8_Cpu_io_DataOut;
  assign _zz_19_ = 1'b1;
  assign core8_serialDataRX = core8_UartRx_io_data_out;
  assign core8_serialDataPresent = core8_UartRx_io_buffer_data_present;
  assign _zz_18_ = (core8_reset || _zz_56_[11]);
  assign _zz_20_ = (core8_buffer_read && (! core8_buffer_read_last));
  assign core8_DMADataOut = (8'b00000000);
  assign _zz_21_ = 1'b1;
  assign avr_rx = core8_UartTx_io_serial_out;
  assign _zz_3_ = ((! core8_Cpu_io_MRD) && (core8_Cpu_io_N == (3'b001)));
  assign core8_serialDataSend = ((! _zz_3_) && _zz_3__regNext);
  always @ (*) begin
    if(((core8_Cpu_io_SC == (2'b10)) && (! _zz_57_[8])))begin
      _zz_14_ = core8_DMADataIN;
    end else begin
      if(((core8_Cpu_io_SC == (2'b10)) && (core8_Cpu_io_N == (3'b000))))begin
        _zz_14_ = core8_ram4096_io_douta;
      end else begin
        if(((! core8_Cpu_io_MWR) && (core8_Cpu_io_N == (3'b010))))begin
          _zz_14_ = _zz_58_[7 : 0];
        end else begin
          if(((! core8_Cpu_io_MWR) && (core8_Cpu_io_N == (3'b001))))begin
            _zz_14_ = core8_serialDataRX;
          end else begin
            _zz_14_ = core8_ram4096_io_douta;
          end
        end
      end
    end
  end

  assign LEDs = {core8_UartRx_io_buffer_data_present,{(core8_buffer_read && (! core8_buffer_read_last)),core8_Cpu_io_Addr16[5 : 0]}};
  always @ (*) begin
    if(_zz_59_[10])begin
      _zz_8_ = 1'b1;
    end else begin
      _zz_8_ = core8_step;
    end
  end

  assign _zz_9_ = _zz_60_[11];
  assign _zz_4_ = core8_Cpu_io_SC[0];
  always @ (posedge core8_clk) begin
    if(core8_reset) begin
      core8_step <= 1'b0;
      core8_stepDMAIn <= 1'b0;
      core8_DMADataIN <= (8'b00000000);
      core8_interrupt <= 1'b0;
      core8_debounce_timer_state <= 1'b0;
      core8_debounce_timer_counter_value <= (19'b0000000000000000000);
      core8_debounce_input <= (12'b000000000000);
      core8_debounce_debounce <= (12'b000000000000);
      core8_debounce_value <= (12'b000000000000);
      core8_SevenSegment_timer_state <= 1'b0;
      core8_SevenSegment_timer_counter_value <= (10'b0000000000);
      core8_SevenSegment_displayCounter_value <= (2'b00);
      core8_buffer_read <= 1'b0;
    end else begin
      core8_debounce_timer_counter_value <= core8_debounce_timer_counter_valueNext;
      if(core8_debounce_timer_counter_willOverflow)begin
        core8_debounce_timer_state <= 1'b1;
      end
      core8_debounce_value <= core8_debounce_debounce;
      if(core8_debounce_timer_state)begin
        core8_debounce_debounce <= core8_debounce_input;
        core8_debounce_timer_state <= 1'b0;
      end
      core8_debounce_input <= switches;
      core8_SevenSegment_timer_counter_value <= core8_SevenSegment_timer_counter_valueNext;
      if(core8_SevenSegment_timer_counter_willOverflow)begin
        core8_SevenSegment_timer_state <= 1'b1;
      end
      core8_SevenSegment_displayCounter_value <= core8_SevenSegment_displayCounter_valueNext;
      if(core8_SevenSegment_timer_state)begin
        core8_SevenSegment_timer_state <= 1'b0;
      end
      if((_zz_62_[9] && _zz_63_[11]))begin
        core8_step <= 1'b1;
      end else begin
        if((_zz_4_ ^ _zz_4__regNext))begin
          core8_step <= 1'b0;
        end
      end
      if((_zz_64_[9] && (! _zz_65_[11])))begin
        core8_stepDMAIn <= 1'b1;
        core8_DMADataIN <= _zz_66_[7 : 0];
        core8_buffer_read <= 1'b0;
      end else begin
        if(((core8_serialDataPresent && (! core8_serialDataPresent_regNext)) && (! _zz_67_[11])))begin
          core8_DMADataIN <= core8_serialDataRX;
          core8_stepDMAIn <= 1'b1;
          core8_buffer_read <= 1'b1;
        end else begin
          if((((! core8_Cpu_io_MWR) && (core8_Cpu_io_N == (3'b001))) && core8_Cpu_io_TPB))begin
            core8_buffer_read <= 1'b1;
          end else begin
            if((core8_Cpu_io_SC == (2'b10)))begin
              core8_stepDMAIn <= 1'b0;
              core8_buffer_read <= 1'b0;
            end else begin
              core8_buffer_read <= 1'b0;
            end
          end
        end
      end
      if(((_zz_68_[11] && _zz_69_[10]) && _zz_70_[9]))begin
        core8_interrupt <= 1'b1;
      end else begin
        if((_zz_71_[11] && core8_serialDataPresent))begin
          core8_interrupt <= 1'b1;
        end else begin
          if((core8_Cpu_io_SC == (2'b11)))begin
            core8_interrupt <= 1'b0;
          end
        end
      end
    end
  end

  always @ (posedge core8_clk) begin
    if(((! (core8_Cpu_io_MRD && core8_Cpu_io_MWR)) && core8_Cpu_io_TPB))begin
      core8_SevenSegment_digits_0 <= _zz_53_[3:0];
      core8_SevenSegment_digits_1 <= _zz_54_;
    end
    core8_buffer_read_last <= core8_buffer_read;
    _zz_3__regNext <= _zz_3_;
    _zz_4__regNext <= _zz_4_;
    core8_serialDataPresent_regNext <= core8_serialDataPresent;
  end

endmodule

