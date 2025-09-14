package my_utils

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import coupledL2.tl2chi.HasCHIOpcodes
import utility.{FastArbiter}
import chisel3.experimental.{noPrefix}
import freechips.rocketchip.diplomacy._

class PriorityEncoderBasicModule(n: Int) extends Module {
  val io = IO(new Bundle {
    val in  = Input(Vec(n, Bool()))
    val idx = Output(UInt(log2Ceil(n).W))
  })
  io.idx := PriorityEncoder(io.in)
}

class GroupPriorityEncoderModule(n:Int, group:Int = 4) extends Module {
    val io = IO(new Bundle() {
        val in = Input(Vec(n, Bool()))
        val idx = Output(UInt(log2Ceil(n).W))
    })

    val g = (n + group - 1) / group
    val paddedN = g * group
    val inPad = Wire(Vec(paddedN, Bool()))
    for (i <- 0 until paddedN) {
        if (i < n) inPad(i) := io.in(i)
        else inPad(i) := false.B
    }
    val grpValid = Wire(Vec(g, Bool()))
    val grpIdxLo = Wire(Vec(g, UInt(log2Ceil(group).W)))
    for (gi <- 0 until g) {
         val base = gi * group 
         val slice = inPad.slice(base, base + group) 
         grpValid(gi) := slice.reduce(_||_) 
         grpIdxLo(gi) := PriorityEncoder(VecInit(slice)) 
    } 
    val anyValid = grpValid.reduce(_||_) 
    val topIdx = PriorityEncoder(grpValid) 
    val idxLo = Mux1H(UIntToOH(topIdx, g), grpIdxLo) 

    val idxCat = Cat(topIdx, idxLo) 
    val rawIdx = idxCat(log2Ceil(n)-1, 0) 
    val safeIdx = Mux(anyValid, rawIdx, 0.U) 

    io.idx := safeIdx 
}


object PriorityEncoderModule {

  def apply(
    in: Seq[Bool],
    impl: String = "flat",
    group: Int   = 4,
    instName: String = "prioEnc"  
  )(implicit vn: ValName): UInt = {
    val n = in.length

    val out = Wire(UInt(log2Ceil(n).W))
    out.suggestName(vn.name)        
    dontTouch(out)

    noPrefix {
      implicit val vnForModule: ValName = ValName(instName)

      val idx = impl match {
        case "flat" =>
          val m = Module(new PriorityEncoderBasicModule(n)) 
          m.io.in := in
          m.io.idx
        case "group"  =>
          val m = Module(new GroupPriorityEncoderModule(n, group)) 
          m.io.in := in
          m.io.idx
        case other =>
          throw new Exception(s"Unknown PE impl: $other")
      }

      out := idx
    }

    out
  }
}
