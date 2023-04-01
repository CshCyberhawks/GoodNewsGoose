package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.commands.auto.arm.AutoArmPosition
import frc.robot.commands.auto.arm.align.ArmAlignClosed
import frc.robot.commands.auto.arm.align.ArmAlignMid
import frc.robot.commands.auto.arm.align.ArmAlignTop
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ExtensionPosition
import frc.robot.util.ControllerIO

/**
 * @property subsystem
 */
class ManualArmCommand(private val subsystem: ArmSystem) : CommandBase() {
    private var currentCommand: CommandBase? = null

    /**
     * Creates a new ExampleCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
//        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = subsystem.armAngleDegrees

//        setCurrentCommand(AutoArmPosition(subsystem, 90.0, 0.0, false, true))
    }

    private fun setCurrentCommand(command: CommandBase) {
        this.currentCommand = command
        this.currentCommand?.schedule()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        SmartDashboard.putBoolean("is cmd null", this.currentCommand == null)

        if (ControllerIO.commandCancel) {
            currentCommand?.cancel()
            currentCommand = null
        }

        SmartDashboard.putBoolean("Command Is Finished", currentCommand?.isFinished == true)

        if (currentCommand != null && currentCommand?.isFinished == false) {
            SmartDashboard.putBoolean("return", true)
            return
        } else if (currentCommand != null && currentCommand?.isFinished == true) {
            currentCommand?.cancel()
            currentCommand = null
        }

        SmartDashboard.putBoolean("return", false)


        if (ControllerIO.toggleTilt) {
            subsystem.desiredTilt = !subsystem.desiredTilt
        }

        if (ControllerIO.extensionExtended) {
            subsystem.desiredExtensionPosition = 3600.0
//            subsystem.desiredExtensionPosition = ExtensionPosition.EXTENDED
            subsystem.hitSetpoint = false

        }
        if (ControllerIO.extensionRetracted) {
            subsystem.desiredExtensionPosition = 0.0
//            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
            subsystem.hitSetpoint = false
        }

        if (ControllerIO.toggleGrabber) {
            subsystem.desiredClawOpen = !subsystem.desiredClawOpen
        }
        if (ControllerIO.togglePID) {
            subsystem.usePID = false
            subsystem.desiredArmAngle = subsystem.armAngleDegrees
            subsystem.desiredAngleSpeed = ControllerIO.controlArmAngle
        } else {
            subsystem.usePID = true
            subsystem.desiredArmAngle += ControllerIO.controlArmAngle * 2
        }

        if (ControllerIO.armAlignUp) {
            subsystem.desiredArmAngle = if (subsystem.desiredTilt) {
                115.0
            } else {
                85.0
            }
        }


//        if (ControllerIO.armAlignTop) {
//            setCurrentCommand(AutoArmPosition(subsystem, 90.0, 0.0, false, true))
//        }

        if (ControllerIO.armAlignTop) {
            currentCommand = ArmAlignTop(subsystem)
            currentCommand?.schedule()
        } else if (ControllerIO.armAlignMid) {
            currentCommand = ArmAlignMid(subsystem)
            currentCommand?.schedule()
        } else if (ControllerIO.armClose) {
            currentCommand = ArmAlignClosed(subsystem)
            currentCommand?.schedule()
        }

        if (ControllerIO.armAlignDown) {
            subsystem.desiredArmAngle = 35.0
        }

        if (ControllerIO.armAlignClosed) {
            subsystem.desiredArmAngle = 35.0
            subsystem.desiredClawOpen = false
            subsystem.desiredExtensionPosition = 0.0
//            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
            subsystem.desiredTilt = false
            subsystem.hitSetpoint = false
        }

        subsystem.run()
    }

    // Called once the command ends or is interruppted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
