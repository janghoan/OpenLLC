package openLLC.my_utils

import chisel3._
import freechips.rocketchip.diplomacy._

object DebugUtils {
  def keep[T <: Data](x: T, name: String, touch: Boolean = true): T = {
    val w = WireDefault(x)
    w.suggestName(name)
    if (touch) { dontTouch(w) }
    w
  }

  def keep[T <: Data](x: T)(implicit vn: ValName): T = {
    keep(x, vn.name, touch = true)
  }
}