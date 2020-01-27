package Spinal1802

import spinal.core._

class Ram(Name: String, AddrDepth: Int = 8, DataDepth: Int = 8) extends Component {
    val io = new Bundle {
        val clka    = in  Bool
        val ena     = in  Bool
        val wea     = in  Bits(1 bit)
        val addra   = in Bits(AddrDepth bit)
        val douta   = out Bits(DataDepth bit)
        val dina    = in Bits(DataDepth bit)
    }

    io.douta := 0
}
