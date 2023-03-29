package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ExtensionPosition
import frc.robot.util.ControllerIO

/**
 * @property subsystem
 */
class ManualArmCommand(private val subsystem: ArmSystem) : CommandBase() {
    /**
     * Creates a new ExampleCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = subsystem.armAngleDegrees
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (ControllerIO.toggleTilt) {
            subsystem.desiredTilt = !subsystem.desiredTilt
        }

        if (ControllerIO.extensionExtended) {
            subsystem.desiredExtensionPosition = ExtensionPosition.EXTENDED
            subsystem.hitSetpoint = false

        }
        if (ControllerIO.extensionRetracted) {
            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
            subsystem.hitSetpoint = false
        }

        if (ControllerIO.toggleGrabber) {
            subsystem.desiredClawOpen = !subsystem.desiredClawOpen
        }
        if (ControllerIO.togglePID) {
            subsystem.usePID = false
            subsystem.desiredArmAngle = subsystem.armAngleDegrees
            subsystem.desiredAngleSpeed = -ControllerIO.controlArmAngle
        } else {
            subsystem.usePID = true
            subsystem.desiredArmAngle += ControllerIO.controlArmAngle * 2
        }

        if (ControllerIO.armAlignUp) {
            subsystem.desiredArmAngle = if (subsystem.desiredTilt) {
                115.0
            } else {
                90.0
            }
        }

        if (ControllerIO.armAlignDown) {
            subsystem.desiredArmAngle = 35.0
        }

        if (ControllerIO.armAlignClosed) {
            subsystem.desiredArmAngle = 35.0
            subsystem.desiredClawOpen = false
            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
            subsystem.desiredTilt = false
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
