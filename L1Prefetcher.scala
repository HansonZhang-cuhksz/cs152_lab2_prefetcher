// See LICENSE.Berkeley for license details.

/*********************************************************************
 * CS152 Lab 2: Open-Ended Problem 4.2                               *
 *********************************************************************/

package freechips.rocketchip.rocket

import chisel3._
import chisel3.util._
import chisel3.experimental.IntParam

import org.chipsalliance.cde.config._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem.CacheBlockBytes

case object BuildL1Prefetcher extends Field[Option[Parameters => L1Prefetcher]](None)

trait CanHaveL1Prefetcher extends HasHellaCache { this: BaseTile =>
  val module: CanHaveL1PrefetcherModule
  implicit val p: Parameters
  if (p(BuildL1Prefetcher).isDefined) {
    nDCachePorts += 1
  }
}

trait CanHaveL1PrefetcherModule extends HasHellaCacheModule { this: RocketTileModuleImp =>
  val outer: CanHaveL1Prefetcher with HasTileParameters
  implicit val p: Parameters

  val prefetchOpt = p(BuildL1Prefetcher).map(_(p))
  val prefetchPort = prefetchOpt.map { pfu =>
    val dmem = Wire(new HellaCacheIO()(p))
    pfu.io.dmem.nack := dmem.s2_nack
    pfu.io.dmem.req.ready := dmem.req.ready
    dmem.req.valid := pfu.io.dmem.req.valid
    dmem.req.bits.addr := pfu.io.dmem.req.bits.addr
    dmem.req.bits.idx.foreach(_ := dmem.req.bits.addr)
    dmem.req.bits.tag := DontCare
    dmem.req.bits.cmd := Mux(pfu.io.dmem.req.bits.write, M_PFW, M_PFR)
    dmem.req.bits.size := log2Ceil(xLen/8).U
    dmem.req.bits.signed := false.B
    dmem.req.bits.phys := false.B
    dmem.req.bits.no_alloc := false.B
    dmem.req.bits.no_xcpt := false.B
    dmem.req.bits.data := DontCare
    dmem.req.bits.dv := DontCare
    dmem.req.bits.mask := 0.U
    dmem.s1_data.data := DontCare
    dmem.s1_data.mask := 0.U
    // Tie off unused control signals
    dmem.s1_kill := false.B
    dmem.s2_kill := false.B
    dmem.keep_clock_enabled := pfu.io.dmem.req.valid
    pfu.io.dmem.resp.valid := dmem.resp.valid // Memory Value Resp
    pfu.io.dmem.resp.bits  := dmem.resp.bits
    dmem
  }
}

// See MemoryOpConstants for encoding of `cmd` field
class CoreMemReq(implicit p: Parameters) extends CoreBundle()(p) with HasCoreMemOp

class L1PrefetchReq(implicit p: Parameters) extends CoreBundle()(p) {
  val addr = UInt(coreMaxAddrBits.W) // Must be aligned to XLEN
  val write = Bool()
}

class L1PrefetcherIO(implicit p: Parameters) extends CoreBundle()(p) {
  val cpu = new Bundle {
    val req = Flipped(Valid(new CoreMemReq))
    val miss = Input(Bool()) // Core request from 2 cycles ago has missed
  }
  val dmem = new Bundle {
    val req = Decoupled(new L1PrefetchReq)
    val nack = Input(Bool()) // Prefetch request from 2 cycles ago is rejected
    val resp = Flipped(Valid(new HellaCacheResp))
  }
}

abstract class L1Prefetcher(implicit p: Parameters) extends CoreModule()(p) {
  val io = IO(new L1PrefetcherIO)
}

/**
 * Naive implementation of the one-block prefetch-on-miss scheme
 */
class ExampleL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {
  
  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)
  val s1_addr = s1_req.addr(coreMaxAddrBits-1, lgCacheBlockBytes)
  val s1_addr_next = s1_addr + 16.U // Compute address of next block

  val s2_req = RegNext(s1_req)
  s2_req.addr := Cat(s1_addr_next, 0.U(lgCacheBlockBytes.W))

  // Keep prefetch request active after a miss is signaled if L1D is not
  // immediately ready to accept it
  val prefetch_count = RegInit(0.U(64.W))
  val miss_hold = RegInit(false.B)
  when (io.cpu.miss) {
    miss_hold := true.B
  }
  // Clear flag after a prefetch request goes through or if another
  // memory request is arriving the next cycle
  when (s1_valid || io.dmem.req.ready) {
    miss_hold := false.B
    prefetch_count := prefetch_count + 1.U
  }

  printf(p"prefetch_count = ${prefetch_count}\n")
  io.dmem.req.valid := io.cpu.miss || miss_hold
  io.dmem.req.bits.addr := s2_req.addr
  io.dmem.req.bits.write := isWrite(s2_req.cmd)

  
  dontTouch(io.dmem.nack)
}


/**
 * TODO: Implement your custom prefetcher
 */
class CustomL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {
  // See L1PrefetcherIO bundle for IO port definitions
  io.dmem.req.valid := false.B // FIXME
}

class NextLineL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {
  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)
  val s1_addr = s1_req.addr(coreMaxAddrBits-1, lgCacheBlockBytes)
  val s1_addr_next = s1_addr + 1.U // Compute address of next block

  val s2_valid = RegNext(s1_valid, false.B)
  val s2_req = RegNext(s1_req)
  s2_req.addr := Cat(s1_addr_next, 0.U(lgCacheBlockBytes.W))
  val curr_req_addr = RegInit(0.U(coreMaxAddrBits.W))

  val access_hold = RegInit(false.B)
  when (s2_valid) {
    access_hold := true.B
  }
  when (io.dmem.req.ready) {
    access_hold := false.B
  }

  io.dmem.req.valid := io.cpu.req.valid || access_hold
  io.dmem.req.bits.addr := s2_req.addr
  io.dmem.req.bits.write := isWrite(s2_req.cmd)
  curr_req_addr := s2_req.addr

  
  dontTouch(io.dmem.nack)
}

class DuplexStrideL1Prefetcher(val threshold: UInt)(implicit p: Parameters) extends L1Prefetcher {
  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)

  val s2_valid = RegNext(s1_valid, false.B)
  val s2_req = RegNext(s1_req)

  val access_hold = RegInit(false.B)
  val last_read_addr = RegInit(0.U(coreMaxAddrBits.W))
  val last_write_addr = RegInit(0.U(coreMaxAddrBits.W))
  val read_confidence = RegInit(0.U(32.W))
  val write_confidence = RegInit(0.U(32.W))
  val read_stride = RegInit(0.U(coreMaxAddrBits.W))
  val write_stride = RegInit(0.U(coreMaxAddrBits.W))
  val prefetch_block_addr = RegInit(0.U(coreMaxAddrBits.W))
  val prefetch_is_write = RegInit(false.B)
  val prefetch = WireInit(false.B)

  when (s2_valid) {
    when (s2_req.cmd === M_XRD) {
      val diff = s2_req.addr - last_read_addr
      when (read_confidence === 0.U) {
        read_confidence := 1.U
        read_stride := diff
      } .otherwise {
        when (diff === read_stride) { // Issue Prefetch
          val new_conf = Mux(read_confidence === threshold, threshold, read_confidence + 1.U)
          read_confidence := new_conf

          prefetch := new_conf === threshold
          val prefetch_addr = (s2_req.addr + 3.U * read_stride).asUInt
          prefetch_block_addr := Cat(prefetch_addr(coreMaxAddrBits-1, lgCacheBlockBytes), 0.U(lgCacheBlockBytes.W))
          prefetch_is_write := false.B
        } .otherwise {
          read_confidence := 1.U
          read_stride := diff
        }
      }
      last_read_addr := s2_req.addr
    }
    .otherwise {
      val diff = s2_req.addr - last_write_addr
      when (write_confidence === 0.U) {
        write_confidence := 1.U
        write_stride := diff
      } .otherwise {
        when (diff === write_stride) { // Issue Prefetch
          val new_conf = Mux(write_confidence === threshold, threshold, write_confidence + 1.U)
          write_confidence := new_conf

          prefetch := new_conf === threshold
          val prefetch_addr = (s2_req.addr + 3.U * write_stride).asUInt
          prefetch_block_addr := Cat(prefetch_addr(coreMaxAddrBits-1, lgCacheBlockBytes), 0.U(lgCacheBlockBytes.W))
          prefetch_is_write := true.B
        } .otherwise {
          write_confidence := 1.U
          write_stride := diff
        }
      }
      last_write_addr := s2_req.addr
    }
  }

  val prefetch_count = RegInit(0.U(64.W))
  when (prefetch && s2_valid) {
    access_hold := true.B
  }
  when (io.dmem.req.fire() || io.dmem.nack || io.cpu.req.valid) {
    access_hold := false.B
    when(io.dmem.req.fire()) {
      prefetch_count := prefetch_count + 1.U
    }
  }

  printf(p"prefetch_count = ${prefetch_count}\n")
  io.dmem.req.valid := access_hold
  io.dmem.req.bits.addr := prefetch_block_addr
  io.dmem.req.bits.write := prefetch_is_write

  
  dontTouch(io.dmem.nack)
}

class DuplexMultiStrideL1Prefetcher(val threshold:UInt, val maxConfidence: UInt)(implicit p: Parameters) extends L1Prefetcher {
  // Stage 1: Confidence Calc
  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)

  val last_read_addr = RegInit(0.U(coreMaxAddrBits.W))
  val last_write_addr = RegInit(0.U(coreMaxAddrBits.W))
  val read_confidence = RegInit(0.U(32.W))
  val write_confidence = RegInit(0.U(32.W))
  val read_stride = RegInit(0.U(coreMaxAddrBits.W))
  val write_stride = RegInit(0.U(coreMaxAddrBits.W))
  
  val s1_task_start = RegInit(0.U.asTypeOf(new CoreMemReq))
  val s1_task_num = RegInit(0.U(32.W))
  val s1_task_stride = RegInit(0.U(coreMaxAddrBits.W))
  val s1_new_task = WireInit(false.B)

  when (s1_valid) {
    when (s1_req.cmd === M_XRD) {
      val diff = s1_req.addr - last_read_addr
      when (read_confidence === 0.U) {
        read_confidence := 1.U
        read_stride := diff
      } .otherwise {
        when (diff === read_stride) {
          val new_conf = Mux(read_confidence === maxConfidence, read_confidence, read_confidence + 1.U)
          read_confidence := new_conf
        
          when (new_conf > threshold) {
            s1_task_start := s1_req
            s1_task_num := new_conf - threshold
            s1_task_stride := read_stride
            s1_new_task := true.B
          }
        } .otherwise {
          read_confidence := 1.U
          read_stride := diff

          s1_task_num := 0.U
        }
      }
      last_read_addr := s1_req.addr
    }
    .otherwise {
      val diff = s1_req.addr - last_write_addr
      when (write_confidence === 0.U) {
        write_confidence := 1.U
        write_stride := diff
      } .otherwise {
        when (diff === write_stride) {
          val new_conf = Mux(write_confidence === maxConfidence, write_confidence, write_confidence + 1.U)
          write_confidence := new_conf

          when (new_conf > threshold) {
            s1_task_start := s1_req
            s1_task_num := new_conf - threshold
            s1_task_stride := write_stride
            s1_new_task := true.B
          }
        } .otherwise {
          write_confidence := 1.U
          write_stride := diff

          s1_task_num := 0.U
        }
      }
      last_write_addr := s1_req.addr
    }
  }

  // Stage 2: Prefetch Issue
  val access_hold = RegInit(false.B)
  val s2_task_start = RegNext(s1_task_start)
  val s2_task_num = RegNext(s1_task_num)
  val s2_task_stride = RegNext(s1_task_stride)
  val s2_new_task = RegNext(s1_new_task)
  val s2_task_done = RegInit(0.U(32.W))

  when (s2_task_num > s2_task_done) {
    access_hold := true.B
  }
  when (io.dmem.req.fire() || io.cpu.req.valid || io.dmem.nack) {
    access_hold := false.B
    when (io.dmem.req.fire()) {
    }
  }

  val prefetch_addr = s2_task_start.addr + (s2_task_done + 1.U) * s2_task_stride
  io.dmem.req.valid := access_hold
  io.dmem.req.bits.addr := Cat(prefetch_addr(coreMaxAddrBits-1, lgCacheBlockBytes), 0.U(lgCacheBlockBytes.W))
  io.dmem.req.bits.write := isWrite(s2_task_start.cmd)
  when (io.dmem.req.fire()) {
    when (s2_task_done < s2_task_num) {
      s2_task_done := s2_task_done + 1.U
    }
  }

  
  dontTouch(io.dmem.nack)
}

class ReadMultiStrideL1Prefetcher(val threshold:UInt, val maxConfidence: UInt)(implicit p: Parameters) extends L1Prefetcher {
  // Stage 1: Confidence Calc
  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)

  val last_read_addr = RegInit(0.U(coreMaxAddrBits.W))
  val read_confidence = RegInit(0.U(32.W))
  val read_stride = RegInit(0.U(coreMaxAddrBits.W))
  
  val s1_task_start = RegInit(0.U.asTypeOf(new CoreMemReq))
  val s1_task_num = RegInit(0.U(32.W))
  val s1_task_stride = RegInit(0.U(coreMaxAddrBits.W))
  val s1_new_task = WireInit(false.B)

  when (s1_valid) {
    when (s1_req.cmd === M_XRD) {
      val diff = s1_req.addr - last_read_addr
      when (read_confidence === 0.U) {
        read_confidence := 1.U
        read_stride := diff
      } .otherwise {
        when (diff === read_stride) {
          val new_conf = Mux(read_confidence === maxConfidence, read_confidence, read_confidence + 1.U)
          read_confidence := new_conf
        
          when (new_conf > threshold) {
            s1_task_start := s1_req
            s1_task_num := new_conf - threshold
            s1_task_stride := read_stride
            s1_new_task := true.B
          }
        } .otherwise {
          read_confidence := 1.U
          read_stride := diff

          s1_task_num := 0.U
        }
      }
      last_read_addr := s1_req.addr
    }
  }

  // Stage 2: Prefetch Issue
  val access_hold = RegInit(false.B)
  val s2_task_start = RegNext(s1_task_start)
  val s2_task_num = RegNext(s1_task_num)
  val s2_task_stride = RegNext(s1_task_stride)
  val s2_new_task = RegNext(s1_new_task)
  val s2_task_done = RegInit(0.U(32.W))

  when (s2_task_num > s2_task_done) {
    access_hold := true.B
  }
  when (io.dmem.req.fire() || io.cpu.req.valid || io.dmem.nack) {
    access_hold := false.B
    when (io.dmem.req.fire()) {
    }
  }

  val prefetch_addr = s2_task_start.addr + (s2_task_done + 1.U) * s2_task_stride
  io.dmem.req.valid := access_hold
  io.dmem.req.bits.addr := Cat(prefetch_addr(coreMaxAddrBits-1, lgCacheBlockBytes), 0.U(lgCacheBlockBytes.W))
  io.dmem.req.bits.write := false.B
  when (io.dmem.req.fire()) {
    when (s2_task_done < s2_task_num) {
      s2_task_done := s2_task_done + 1.U
    }
  }

  
  dontTouch(io.dmem.nack)
}

class WriteMultiStrideL1Prefetcher(val threshold:UInt, val maxConfidence: UInt)(implicit p: Parameters) extends L1Prefetcher {
  // Stage 1: Confidence Calc
  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)

  val last_write_addr = RegInit(0.U(coreMaxAddrBits.W))
  val write_confidence = RegInit(0.U(32.W))
  val write_stride = RegInit(0.U(coreMaxAddrBits.W))
  
  val s1_task_start = RegInit(0.U.asTypeOf(new CoreMemReq))
  val s1_task_num = RegInit(0.U(32.W))
  val s1_task_stride = RegInit(0.U(coreMaxAddrBits.W))
  val s1_new_task = WireInit(false.B)

  when (s1_valid) {
    when (s1_req.cmd === M_XWR) {
      val diff = s1_req.addr - last_write_addr
      when (write_confidence === 0.U) {
        write_confidence := 1.U
        write_stride := diff
      } .otherwise {
        when (diff === write_stride) {
          val new_conf = Mux(write_confidence === maxConfidence, write_confidence, write_confidence + 1.U)
          write_confidence := new_conf
        
          when (new_conf > threshold) {
            s1_task_start := s1_req
            s1_task_num := new_conf - threshold
            s1_task_stride := write_stride
            s1_new_task := true.B
          }
        } .otherwise {
          write_confidence := 1.U
          write_stride := diff

          s1_task_num := 0.U
        }
      }
      last_write_addr := s1_req.addr
    }
  }

  // Stage 2: Prefetch Issue
  val access_hold = RegInit(false.B)
  val s2_task_start = RegNext(s1_task_start)
  val s2_task_num = RegNext(s1_task_num)
  val s2_task_stride = RegNext(s1_task_stride)
  val s2_new_task = RegNext(s1_new_task)
  val s2_task_done = RegInit(0.U(32.W))

  when (s2_task_num > s2_task_done) {
    access_hold := true.B
  }
  when (io.dmem.req.fire() || io.cpu.req.valid || io.dmem.nack) {
    access_hold := false.B
    when (io.dmem.req.fire()) {
    }
  }

  val prefetch_addr = s2_task_start.addr + (s2_task_done + 1.U) * s2_task_stride
  io.dmem.req.valid := access_hold
  io.dmem.req.bits.addr := Cat(prefetch_addr(coreMaxAddrBits-1, lgCacheBlockBytes), 0.U(lgCacheBlockBytes.W))
  io.dmem.req.bits.write := false.B
  when (io.dmem.req.fire()) {
    when (s2_task_done < s2_task_num) {
      s2_task_done := s2_task_done + 1.U
    }
  }

  
  dontTouch(io.dmem.nack)
}

class DMPL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {
  // Stage 1: Preprocess
  // Memory Region Detection
  val s1_valid = RegNext(io.cpu.req.valid, false.B)
  val s1_req = RegEnable(io.cpu.req.bits, io.cpu.req.valid)
  val s1_mem_addr_cap = RegInit(0x7F.U(coreMaxAddrBits.W))
  val s1_mem_addr_floor = RegInit(~0.U(coreMaxAddrBits.W))
  when (s1_valid) {
    when (s1_req.addr > s1_mem_addr_cap) {
      s1_mem_addr_cap := s1_req.addr
    }
    when (s1_req.addr < s1_mem_addr_floor) {
      s1_mem_addr_floor := s1_req.addr
    }
  }
  // Memory Response Retrieval
  val s1_mem_valid = RegNext(io.dmem.resp.valid, false.B)
  val s1_mem_data = RegEnable(io.dmem.resp.bits.data, io.dmem.resp.valid)
  val s1_data_is_addr = RegInit(false.B)
  when (s1_mem_valid) {
    val in_range = (s1_mem_data <= s1_mem_addr_cap) && (s1_mem_data >= s1_mem_addr_floor)
    val is_aligned = s1_mem_data(lgCacheBlockBytes-1, 0) === 0.U
    s1_data_is_addr := in_range && is_aligned
  }

  // Stage 2: Prefetch Issue
  val s2_prefetch = RegNext(s1_data_is_addr, false.B)
  val s2_addr = RegNext(s1_mem_data, 0.U)
  val access_hold = RegInit(false.B)
  when (s2_prefetch && !io.dmem.req.valid) {
    access_hold := true.B
  }
  when (io.dmem.req.fire() || io.cpu.req.valid || io.dmem.nack) {
    access_hold := false.B
  }
  io.dmem.req.valid := access_hold
  io.dmem.req.bits.addr := Cat(s2_addr(coreMaxAddrBits-1, lgCacheBlockBytes), 0.U(lgCacheBlockBytes.W))
  io.dmem.req.bits.write := false.B

  dontTouch(io.dmem.nack)
}

class MultiL1PrefetcherWrapper(implicit p: Parameters) extends L1Prefetcher {
  val nextLine     = Module(new NextLineL1Prefetcher)
  val readStride   = Module(new ReadMultiStrideL1Prefetcher(threshold = 3.U, maxConfidence = 8.U))
  val writeStride  = Module(new WriteMultiStrideL1Prefetcher(threshold = 3.U, maxConfidence = 8.U))
  val dmp          = Module(new DMPL1Prefetcher)

  val subPrefetchers = Seq(readStride, writeStride, dmp, nextLine)

  // Broadcast CPU signals to all sub-prefetchers
  for (pfu <- subPrefetchers) {
    pfu.io.cpu.req   := io.cpu.req
    pfu.io.cpu.miss  := io.cpu.miss
  }

  // Arbitrate dmem requests using a round-robin arbiter
  val arb = Module(new Arbiter(new L1PrefetchReq, subPrefetchers.length))
  for ((pfu, i) <- subPrefetchers.zipWithIndex) {
    arb.io.in(i) <> pfu.io.dmem.req
  }
  io.dmem.req <> arb.io.out

  // Count and print total prefetches issued
  val prefetch_count = RegInit(0.U(64.W))
  when (io.dmem.req.fire()) {
    prefetch_count := prefetch_count + 1.U
  }
  printf(p"prefetch_count = ${prefetch_count}\n")
  printf(p"winner = ${arb.io.chosen}\n")

  val s1_chosen = RegNext(arb.io.chosen)
  val s2_chosen = RegNext(s1_chosen)

  for ((pfu, i) <- subPrefetchers.zipWithIndex) {
    pfu.io.dmem.nack       := io.dmem.nack && (s2_chosen === i.U)
    pfu.io.dmem.resp.valid := io.dmem.resp.valid
    pfu.io.dmem.resp.bits  := io.dmem.resp.bits
  }
}

/**
 * Wrapper around C++ software model
 */
class ModelL1Prefetcher(implicit p: Parameters) extends L1Prefetcher {
  val model = Module(new ModelL1PrefetcherHarness(
    addrBits = io.cpu.req.bits.addr.getWidth,
    tagBits = io.cpu.req.bits.tag.getWidth,
    cmdBits = io.cpu.req.bits.cmd.getWidth,
    sizeBits = io.cpu.req.bits.size.getWidth))

  model.io.clock := clock
  model.io.reset := reset
  model.io.cpu := io.cpu
  io.dmem <> model.io.dmem
}

/**
 * Blackbox for Verilog DPI harness
 */
class ModelL1PrefetcherHarness(addrBits: Int, tagBits: Int, cmdBits: Int, sizeBits: Int)(implicit p: Parameters)
    extends BlackBox(Map(
      "ADDR_BITS" -> IntParam(addrBits),
      "TAG_BITS" -> IntParam(tagBits),
      "CMD_BITS" -> IntParam(cmdBits),
      "SIZE_BITS" -> IntParam(sizeBits),
      "LINE_SHIFT" -> IntParam(log2Up(p(CacheBlockBytes)))))
    with HasBlackBoxResource {

  val io = IO(new L1PrefetcherIO {
    val clock = Input(Clock())
    val reset = Input(Bool())
  })

  addResource("/vsrc/L1Prefetcher.v")
  addResource("/csrc/L1Prefetcher.cc")
}
