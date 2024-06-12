package vexriscv.demo

import spinal.core._
import spinal.lib._
import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.misc.SizeMapping
import spinal.lib.bus.simple.PipelinedMemoryBus
import spinal.lib.com.spi.ddr.SpiXdrMaster
import spinal.lib.com.uart._
import spinal.lib.io.{InOutWrapper, TriStateArray}
import vexriscv.plugin._
import vexriscv.{VexRiscv, VexRiscvConfig, plugin}
import spinal.lib.com.spi.ddr._
import spinal.lib.bus.simple._
import scala.collection.mutable.ArrayBuffer
import scala.collection.Seq
import spinal.lib.blackbox.altera.VJTAG

/* Case class for the Murax SoC config */
case class MuraxConfig(coreFrequency            : HertzNumber,
                       pipelineDBus             : Boolean,
                       pipelineMainBus          : Boolean,
                       pipelineApbBridge        : Boolean,
                       gpioWidth                : Int,
                       hardwareBreakpointCount  : Int,
                       cpuPlugins               : ArrayBuffer[Plugin[VexRiscv]]){
  require(pipelineApbBridge || pipelineMainBus, "At least pipelineMainBus or pipelineApbBridge should be enabled to avoid wipe transactions")
}

/* Companion object for MuraxConfig with default conf */
object MuraxConfig{
  def default : MuraxConfig = default(false)

  /* Default configuration method */
  def default(bigEndian : Boolean = false) =  MuraxConfig(
    coreFrequency         = 12 MHz,
    pipelineDBus          = true,
    pipelineMainBus       = false,
    pipelineApbBridge     = true,
    gpioWidth = 32,
    hardwareBreakpointCount = 0,
    cpuPlugins = ArrayBuffer(

      /* Instruction bus */
      new IBusSimplePlugin(
        resetVector = 0x80040000l,
        cmdForkOnSecondStage = true,
        cmdForkPersistence = false,
        prediction = NONE,
        catchAccessFault = false,
        compressedGen = false,
        bigEndian = bigEndian
      ),

      /* Data bus */
      new DBusSimplePlugin(
        catchAddressMisaligned = false,
        catchAccessFault = false,
        earlyInjection = false,
        bigEndian = bigEndian
      ),

      /* Control state register plugin */
      new CsrPlugin(CsrPluginConfig.smallest(mtvecInit = 0x80000020l)),

      /* Instruction decoder */
      new DecoderSimplePlugin(
        catchIllegalInstruction = false
      ),

      /* Register file */
      new RegFilePlugin(
        regFileReadyKind = plugin.SYNC,
        zeroBoot = false
      ),

      /* Int ALU */
      new IntAluPlugin,

      /* Source plugin */
      new SrcPlugin(
        separatedAddSub = false,
        executeInsertion = false
      ),

      /* Light shifter */
      new LightShifterPlugin,

      /* Hazard simple */
      new HazardSimplePlugin(
        bypassExecute = false,
        bypassMemory = false,
        bypassWriteBack = false,
        bypassWriteBackBuffer = false,
        pessimisticUseSrc = false,
        pessimisticWriteRegFile = false,
        pessimisticAddressMatch = false
      ),

      /* Branch */
      new BranchPlugin(
        earlyBranch = false,
        catchAddressMisaligned = false
      ),

      /* Generate YAML file */
      new YamlPlugin("cpu0.yaml")
    )
  )

  /* Fast configuration method which enables bypasses in the Hazard plugin */
  def fast = {
    val config = default

    config.cpuPlugins(config.cpuPlugins.indexWhere(_.isInstanceOf[HazardSimplePlugin])) = new HazardSimplePlugin(
      bypassExecute = true,
      bypassMemory = true,
      bypassWriteBack = true,
      bypassWriteBackBuffer = true
    )

    config
  }
}

/* Murax SoC class definition */
case class Murax(config : MuraxConfig) extends Component{
  import config._

  /* IO ports for the SoC */
  val io = new Bundle {

    /* Clock and reset */
    val asyncReset = in Bool()
    val mainClk = in Bool()
    
    /* System reset as output */
    val systemReset = out Bool()

    /* Interrupt pins */
    val timerInterrupt = in Bool()
    val externalInterrupt = in Bool()

    /* Exposed APB interface for slave peripherals */
    val apb = master(Apb3(
      Apb3Config(
        addressWidth = 20,
        dataWidth = 32,
        useSlaveError = true
      ))
    ) 
  }

  /* Reset control clock domain */
  val resetCtrlClockDomain = ClockDomain(
    clock = io.mainClk,
    config = ClockDomainConfig(
      resetKind = BOOT
    )
  )

  /* Clocking area for reset control logic */
  val resetCtrl = new ClockingArea(resetCtrlClockDomain) {
    val mainClkResetUnbuffered  = False

    // Counter to ensure reset signal is asserted for enough time
    val systemClkResetCounter = Reg(UInt(6 bits)) init(0)
    when(systemClkResetCounter =/= U(systemClkResetCounter.range -> true)){
      systemClkResetCounter := systemClkResetCounter + 1
      mainClkResetUnbuffered := True
    }
    when(BufferCC(io.asyncReset)){
      systemClkResetCounter := 0
    }

    val mainClkReset = RegNext(mainClkResetUnbuffered)
    val systemReset  = RegNext(mainClkResetUnbuffered)
  }

  /* Main system clock domain */
  val systemClockDomain = ClockDomain(
    clock = io.mainClk,
    reset = resetCtrl.systemReset,
    frequency = FixedFrequency(coreFrequency)
  )

  /* Debug clock domain */
  val debugClockDomain = ClockDomain(
    clock = io.mainClk,
    reset = resetCtrl.mainClkReset,
    frequency = FixedFrequency(coreFrequency)
  )

  /* Main system logic in the system clock domain */
  val system = new ClockingArea(systemClockDomain) {
    val pipelinedMemoryBusConfig = PipelinedMemoryBusConfig(
      addressWidth = 32,
      dataWidth = 32
    )

    /* Data bus */
    val bigEndianDBus = config.cpuPlugins.exists {
      case plugin: DBusSimplePlugin => plugin.bigEndian
      case _ => false
    }

    /* Arbiter of the CPU dBus/iBus to drive the main bus */
    /* Priority to dBus */
    val mainBusArbiter = new MuraxMasterArbiter(pipelinedMemoryBusConfig, bigEndianDBus)

    /* Instanciate the CPU and add the debug plugin */
    val cpu = new VexRiscv(
      config = VexRiscvConfig(
        plugins = cpuPlugins += new DebugPlugin(debugClockDomain, hardwareBreakpointCount)
      )
    )

    /* Connect system reset to output pin */
    io.systemReset <> resetCtrl.systemReset
    
    /* Map interrupts to the IO pins */
    val timerInterrupt = io.timerInterrupt
    val externalInterrupt = io.externalInterrupt
    val softwareInterrupt = false

    /* Connect CPU plugins to the SoC */
    for(plugin <- cpu.plugins) plugin match{

      /* Instruction bus */
      case plugin : IBusSimplePlugin =>
        mainBusArbiter.io.iBus.cmd <> plugin.iBus.cmd
        mainBusArbiter.io.iBus.rsp <> plugin.iBus.rsp

      /* Data bus */
      case plugin : DBusSimplePlugin => {
        if(!pipelineDBus)
          mainBusArbiter.io.dBus <> plugin.dBus
        else {
          mainBusArbiter.io.dBus.cmd << plugin.dBus.cmd.halfPipe()
          mainBusArbiter.io.dBus.rsp <> plugin.dBus.rsp
        }
      }

      /* Control state register */
      case plugin : CsrPlugin        => {
        plugin.externalInterrupt := externalInterrupt
        plugin.timerInterrupt := timerInterrupt
      }

      /* Debug using virtual JTAG */
      case plugin : DebugPlugin         => plugin.debugClockDomain{
        resetCtrl.systemReset setWhen(RegNext(plugin.io.resetOut))
        plugin.io.bus.fromVJtag()
      }
      case _ =>
    }

    /* APB bridge */
    val apbBridge = new PipelinedMemoryBusToApbBridge(
      apb3Config = Apb3Config(
        addressWidth = 20,
        dataWidth = 32
      ),
      pipelineBridge = pipelineApbBridge,
      pipelinedMemoryBusConfig = pipelinedMemoryBusConfig
    )
    
    /* Connect APB bridge to IO signals */
    io.apb <> apbBridge.io.apb

    /* Main bus slaves */
    val mainBusMapping = ArrayBuffer[(PipelinedMemoryBus,SizeMapping)]()
    mainBusMapping += apbBridge.io.pipelinedMemoryBus -> (0x80000000l, 1 MB)

    /* Main bus decoder to handle address mapping and pipelining */
    val mainBusDecoder = new Area {
      val logic = new MuraxPipelinedMemoryBusDecoder(
        master = mainBusArbiter.io.masterBus,
        specification = mainBusMapping.toSeq,
        pipelineMaster = pipelineMainBus
      )
    }
  }
}

/* Main object to generate the VHDL description */
object Murax{
  /* Generates Murax.vhd file */
  def main(args: Array[String]) {
    SpinalVhdl(Murax(MuraxConfig.default))
  }
}