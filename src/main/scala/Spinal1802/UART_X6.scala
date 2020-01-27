package Spinal1802

import spinal.core._

class uart_rx6(Name: String) extends Component {
    val io = new Bundle {
        val clk = in Bool
        val buffer_reset = in Bool
        val serial_in = in Bool
        val en_16_x_baud = in Bool
        val buffer_read = in Bool

        val buffer_data_present = out Bool
        val buffer_half_full = out Bool
        val buffer_full = out Bool
        val data_out = out Bits(8 bits)
    }

    io.buffer_data_present := False
    io.buffer_half_full := False
    io.buffer_full := False
    io.data_out := 0
}

class uart_tx6(Name: String) extends Component {
    val io = new Bundle {
        val clk = in Bool
        val buffer_reset = in Bool
        val data_in = in Bits(8 bits)
        val en_16_x_baud = in Bool
        val buffer_write = in Bool

        val serial_out= out Bool
        val buffer_data_present = out Bool
        val buffer_half_full = out Bool
        val buffer_full = out Bool
    }

    io.serial_out := False
    io.buffer_data_present := False
    io.buffer_half_full := False
    io.buffer_full := False
}
