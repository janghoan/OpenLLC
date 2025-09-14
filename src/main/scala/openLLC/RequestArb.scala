/** *************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 * http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 * *************************************************************************************
 */

package openLLC

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import coupledL2.tl2chi.HasCHIOpcodes
import scala.math.min
import freechips.rocketchip.diplomacy._
import chisel3.experimental.noPrefix
import my_utils.DebugUtils._


class RequestArb(implicit p: Parameters) extends LLCModule with HasClientInfo with HasCHIOpcodes {
  val io = IO(new Bundle() {
    /* receive incoming tasks from s1 */
    val busTask_s1 = Flipped(DecoupledIO(new Task()))
    val refillTask_s1 = Flipped(DecoupledIO(new Task()))

    /* read local/client directory */
    val dirRead_s1 = DecoupledIO(new DirRead())

    /* send task to mainPipe */
    val taskToPipe_s2 = ValidIO(new Task())

    /* send refillBuf read request */
    val refillBufRead_s2 = ValidIO(new RefillBufRead())

    /* handle set conflict, capacity conflict and coherency conflict */
    val pipeInfo = Input(new PipeStatus())
    val refillInfo = Flipped(Vec(mshrs.refill, ValidIO(new BlockInfo())))
    val respInfo = Flipped(Vec(mshrs.response, ValidIO(new ResponseInfo())))
    val snpInfo = Flipped(Vec(mshrs.snoop, ValidIO(new BlockInfo())))
    val memInfo = Flipped(Vec(mshrs.memory, ValidIO(new MemInfo())))
  })

  val pipeInfo   = io.pipeInfo
  val refillInfo = io.refillInfo
  val respInfo   = io.respInfo
  val snpInfo    = io.snpInfo
  val memInfo    = io.memInfo

  val task_s1 = Wire(Valid(new Task()))
  val task_s2 = Wire(Valid(new Task()))

  /* Stage 1 */
  def minSetBits = min(setBits, clientSetBits)

  val tag_s1   = task_s1.bits.tag
  val set_s1   = task_s1.bits.set
  val reqID_s1 = task_s1.bits.reqID
  val potentialCount = PopCount(pipeInfo.valids)

  val isBusTask = !task_s1.bits.refillTask
  val isRefillTask = task_s1.bits.refillTask

  def addrConnect(lset: UInt, ltag: UInt, rset: UInt, rtag: UInt) = {
    assert(lset.getWidth + ltag.getWidth == rset.getWidth + rtag.getWidth)
    val addr = Cat(rtag, rset)
    lset := addr.tail(ltag.getWidth)
    ltag := addr.head(ltag.getWidth)
  }


  private def checkAddrMatch[T<:Data](info: Seq[ValidIO[T]], task_tag: UInt, task_set: UInt)
                            (getTag: T => UInt, getSet: T => UInt) // getter
                            (implicit vn: ValName): Bool = {
    val checker = info.map(e => e.valid &&(getTag(e.bits) === task_tag) && (getSet(e.bits) === task_set))
    val res = WireDefault(VecInit(checker).asUInt.orR).suggestName(vn.name)
    dontTouch(res)
    res
  }

  private def checkAddrOpMatch[T<:Data](info: Seq[ValidIO[T]], task_tag: UInt, task_set: UInt)
                            (getTag: T => UInt, getSet: T => UInt, getOpcode:T=>UInt, getWdatRsp: T=>UInt) // getter
                            (implicit vn: ValName): Bool = {
    val checker = info.map(e => e.valid && (getTag(e.bits) === task_tag) && (getSet(e.bits) === task_set) && 
     getOpcode(e.bits) === WriteNoSnpFull && (isRefillTask || isClean_s1 || !getWdatRsp(e.bits) && isRead_s1))
    val res = WireDefault(VecInit(checker).asUInt.orR).suggestName(vn.name)
    dontTouch(res)
    res
  }

  private def checkReqIDMatch[T<:Data](info: Seq[ValidIO[T]], task_reqID: UInt)
                          (getReqID:T=>UInt)
                          (implicit vn: ValName): Bool = {
    val checker = info.map(e => e.valid && getReqID(e.bits) === task_reqID)
    val res = WireDefault(VecInit(checker).asUInt.orR).suggestName(vn.name)
    dontTouch(res)
    res
  }

  private def checkMSHRFull(inflight: UInt, potential: UInt, mshr: UInt)(implicit vn: ValName): Bool = {
    val full = inflight >= mshr
    val room    = Mux(full, 0.U, mshr - inflight) 
    val res = WireDefault(full || (room <= potential)).suggestName(vn.name)
    dontTouch(res)
    res

  }

  val isReadNotSharedDirty_s1 = isBusTask && task_s1.bits.chiOpcode === ReadNotSharedDirty
  val isReadUnique_s1 = isBusTask && task_s1.bits.chiOpcode === ReadUnique
  val isCleanInvalid_s1 = isBusTask && task_s1.bits.chiOpcode === CleanInvalid
  val isCleanShared_s1 = isBusTask && task_s1.bits.chiOpcode === CleanShared
  val isWriteCleanFull_s1 = isBusTask && task_s1.bits.chiOpcode === WriteCleanFull

  val isRead_s1  = isReadNotSharedDirty_s1 || isReadUnique_s1
  val isClean_s1 = isCleanInvalid_s1 || isCleanShared_s1 || isWriteCleanFull_s1

  // To prevent data hazards caused by read-after-write conflicts in the directory,
  // blocking is required when the set of s1 is the same as that of s2 or s3
  val sameSet_s2 = pipeInfo.s2_valid && pipeInfo.s2_set(minSetBits - 1, 0) === set_s1(minSetBits - 1, 0)
  val sameSet_s3 = pipeInfo.s3_valid && pipeInfo.s3_set(minSetBits - 1, 0) === set_s1(minSetBits - 1, 0)
  // Snoop tasks caused by replacements may be issued at S4 stage,
  // so blocking is required when the set of S1 matches S4.
  val sameSet_s4  = pipeInfo.s4_valid && pipeInfo.s4_set(minSetBits - 1, 0) === set_s1(minSetBits - 1, 0)
  val sameAddr_s5 = pipeInfo.s5_valid && Cat(pipeInfo.s5_tag, pipeInfo.s5_set) === Cat(tag_s1, set_s1)
  val sameAddr_s6 = pipeInfo.s6_valid && Cat(pipeInfo.s6_tag, pipeInfo.s6_set) === Cat(tag_s1, set_s1)

  val sameReqID_s2 = pipeInfo.s2_valid && pipeInfo.s2_reqID === reqID_s1
  val sameReqID_s3 = pipeInfo.s3_valid && pipeInfo.s3_reqID === reqID_s1
  val sameReqID_s4 = pipeInfo.s4_valid && pipeInfo.s4_reqID === reqID_s1
  val sameReqID_s5 = pipeInfo.s5_valid && pipeInfo.s5_reqID === reqID_s1
  val sameReqID_s6 = pipeInfo.s6_valid && pipeInfo.s6_reqID === reqID_s1

  // Since the stages within the MainPipe are non-blocking, when the sum of the requests being processed
  // in the buffer and the potential requests that might occupy the buffer in the MainPipe exceeds the 
  // total number of buffer entries, new requests need to be blocked from entering the MainPipe.
  val inflight_refill    = PopCount(refillInfo.map(e => e.valid))
  val inflight_snoop     = PopCount(snpInfo.map(e => e.valid))
  val inflight_response  = PopCount(respInfo.map(e => e.valid))
  val inflight_memAccess = PopCount(memInfo.map(e => e.valid))
  val potential_refill = Wire(UInt())
  dontTouch(potential_refill)
  val pipeInfo_s2_valid = keep(pipeInfo.s2_valid)
  val pipeInfo_s3_valid = keep(pipeInfo.s3_valid)
  val pipeInfo_s4_valid = keep(pipeInfo.s4_valid)
  potential_refill := PopCount(Cat(pipeInfo_s2_valid, pipeInfo_s3_valid, pipeInfo_s4_valid))
  val potential_response = keep(potentialCount)
  val potential_memAccess = keep(potentialCount)

  /* BLOCK BY MAINPIPE */
  val mainPipeSameSet = sameSet_s2 || sameSet_s3 || sameSet_s4
  val mainPipeSameAddr = sameAddr_s5 || sameAddr_s6
  val mainPipeSameReqID = sameReqID_s2 || sameReqID_s3 || sameReqID_s4 || sameReqID_s5 || sameReqID_s6
  val blockByMainPipe = mainPipeSameSet || mainPipeSameAddr || mainPipeSameReqID
  // val blockByMainPipe = sameSet_s2 || sameSet_s3 || sameSet_s4 || sameAddr_s5 || sameAddr_s6 ||
  //   sameReqID_s2 || sameReqID_s3 || sameReqID_s4 || sameReqID_s5 || sameReqID_s6
  dontTouch(blockByMainPipe)
  
  /* BLOCK BY REFILL */
  val refillTask_condition = isBusTask
  val refillAddrMatch = checkAddrMatch(refillInfo, tag_s1, set_s1)(_.tag, _.set)
  val refillReqIDMatch = checkReqIDMatch(refillInfo, reqID_s1)(_.reqID)
  val refillCapFull = checkMSHRFull(inflight_refill, potential_refill, mshrs.refill.U)
  val blockByRefill = refillTask_condition && (refillAddrMatch || refillReqIDMatch || refillCapFull)
  dontTouch(blockByRefill)
  // val blockByRefill = !task_s1.bits.refillTask && (
  //   Cat(refillInfo.map(e => e.valid && Cat(e.bits.tag, e.bits.set) === Cat(tag_s1, set_s1))).orR ||
  //   Cat(refillInfo.map(e => e.valid && e.bits.reqID === reqID_s1)).orR ||
  //   (inflight_refill +& potential_refill) >= mshrs.refill.U
  // )

  /* BLOCK BY RESPONSE */
  val respTask_condition = isBusTask
  val respAddrMatch = checkAddrMatch(respInfo, tag_s1, set_s1)(_.tag, _.set)
  val respReqIDMatch = checkReqIDMatch(respInfo, reqID_s1)(_.reqID)
  val respCapFull = checkMSHRFull(inflight_response, potential_response, mshrs.response.U)
  val blockByResp = respTask_condition && (respAddrMatch || respReqIDMatch || respCapFull)
  dontTouch(blockByResp)
  /* BLOCK BY SNOOP */
  // val blockByResp = !task_s1.bits.refillTask && (
  //   Cat(respInfo.map(e => e.valid && Cat(e.bits.tag, e.bits.set) === Cat(tag_s1, set_s1))).orR ||
  //   Cat(respInfo.map(e => e.valid && e.bits.reqID === reqID_s1)).orR ||
  //   (inflight_response +& potential_response) >= mshrs.response.U
  // )
  
  // val blockBySnp = !task_s1.bits.refillTask && (
  //   Cat(snpInfo.map(e => e.valid && e.bits.reqID === reqID_s1)).orR ||
  //   (inflight_snoop +& potential_snoop) >= mshrs.snoop.U
  // )
  /* BLOCK BY MEM */
  val memAddrOpMatch = checkAddrOpMatch(memInfo, tag_s1, set_s1)(_.tag, _.set, _.opcode, _.w_datRsp)
  val memReqIDMatch = noPrefix{checkReqIDMatch(memInfo, reqID_s1)(_.reqID) && isBusTask}
  val memCapFull = checkMSHRFull(inflight_memAccess , potential_memAccess, mshrs.memory.U)
  val blockByMem = memAddrOpMatch || memReqIDMatch || memCapFull
  dontTouch(blockByMem)
  // val blockByMem = Cat(memInfo.map(e => e.valid && Cat(e.bits.tag, e.bits.set) === Cat(tag_s1, set_s1) &&
  //   e.bits.opcode === WriteNoSnpFull && (task_s1.bits.refillTask || isClean_s1 || !e.bits.w_datRsp && isRead_s1))).orR ||
  //   Cat(memInfo.map(e => e.valid && e.bits.reqID === reqID_s1 && !task_s1.bits.refillTask)).orR ||
  //   (inflight_memAccess +& potential_memAccess) >= mshrs.memory.U

  val blockEntrance = blockByMainPipe || blockByRefill || blockByResp || blockByMem

  task_s1.valid := io.dirRead_s1.ready && (io.busTask_s1.valid || io.refillTask_s1.valid) && !blockEntrance
  task_s1.bits := Mux(io.refillTask_s1.valid, io.refillTask_s1.bits, io.busTask_s1.bits)

  io.busTask_s1.ready := io.dirRead_s1.ready && !io.refillTask_s1.valid && !blockEntrance
  io.refillTask_s1.ready := io.dirRead_s1.ready && !blockEntrance


  // Meta read request
  io.dirRead_s1.valid := task_s1.valid
  val rports = Seq(io.dirRead_s1.bits.self, io.dirRead_s1.bits.clients)
  rports.foreach { p =>
    addrConnect(p.set, p.tag, set_s1, tag_s1)
    p.replacerInfo.opcode := task_s1.bits.chiOpcode
    p.replacerInfo.refill := task_s1.bits.refillTask
  }

  /* Stage 2 */
  task_s2.valid := RegNext(task_s1.valid, false.B)
  task_s2.bits := RegEnable(task_s1.bits, task_s1.valid)

  io.taskToPipe_s2 := task_s2

  io.refillBufRead_s2.valid := task_s2.valid && task_s2.bits.refillTask
  io.refillBufRead_s2.bits.id := task_s2.bits.bufID

}
