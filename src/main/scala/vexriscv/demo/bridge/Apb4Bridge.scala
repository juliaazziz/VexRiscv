package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba4.apb._
import spinal.lib.bus.simple.PipelinedMemoryBus
import spinal.lib.bus.simple._

case class PipelinedMemoryBusToApb4Bridge(apb4Config: Apb4Config, pipelineBridge : Boolean, pipelinedMemoryBusConfig : PipelinedMemoryBusConfig) extends Component{
  assert(apb4Config.dataWidth == pipelinedMemoryBusConfig.dataWidth)

  val io = new Bundle {
    val pipelinedMemoryBus = slave(PipelinedMemoryBus(pipelinedMemoryBusConfig))
    val apb = master(Apb4(apb4Config))
  }

  val pipelinedMemoryBusStage = PipelinedMemoryBus(pipelinedMemoryBusConfig)
  pipelinedMemoryBusStage.cmd << (if(pipelineBridge) io.pipelinedMemoryBus.cmd.halfPipe() else io.pipelinedMemoryBus.cmd)
  pipelinedMemoryBusStage.rsp >-> io.pipelinedMemoryBus.rsp

  val state = RegInit(False)
  pipelinedMemoryBusStage.cmd.ready := False

  io.apb.PSEL(0) := pipelinedMemoryBusStage.cmd.valid
  io.apb.PENABLE := state
  io.apb.PWRITE  := pipelinedMemoryBusStage.cmd.write
  io.apb.PADDR   := pipelinedMemoryBusStage.cmd.address.resized
  io.apb.PWDATA  := pipelinedMemoryBusStage.cmd.data
  io.apb.PSTRB  := pipelinedMemoryBusStage.cmd.mask
  io.apb.PPROT  := B"000"

  pipelinedMemoryBusStage.rsp.valid := False
  pipelinedMemoryBusStage.rsp.data  := io.apb.PRDATA
  when(!state) {
    state := pipelinedMemoryBusStage.cmd.valid
  } otherwise {
    when(io.apb.PREADY){
      state := False
      pipelinedMemoryBusStage.rsp.valid := !pipelinedMemoryBusStage.cmd.write
      pipelinedMemoryBusStage.cmd.ready := True
    }
  }
}
