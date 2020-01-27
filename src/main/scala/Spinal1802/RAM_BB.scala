package Spinal1802

import spinal.core._

class Ram(Name: String, AddrDepth: Int = 8, DataDepth: Int = 8) extends Component {
    val io = new Bundle {
        val ena     = in  Bool
        val wea     = in  Bits(1 bit)
        val addra   = in Bits(AddrDepth bit)
        val douta   = out Bits(DataDepth bit)
        val dina    = in Bits(DataDepth bit)
    }

    val mem = Mem(Bits(DataDepth bits), wordCount = (1 << AddrDepth))

    mem.write(
      enable = io.ena & io.wea.asBool,
      address = io.addra.asUInt,
      data = io.dina
    )

    io.douta := mem.readSync(
      enable = io.ena,
      address = io.addra.asUInt
    )
}
