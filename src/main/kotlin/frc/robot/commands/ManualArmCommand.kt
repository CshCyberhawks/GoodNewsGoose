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
        subsystem.desiredArmAngle += ControllerIO.controlArmAngle * 2

        if (ControllerIO.extensionExtended) {
            subsystem.desiredExtensionPosition = ExtensionPosition.EXTENDED
        }
        if (ControllerIO.extensionRetracted) {
            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
        }

        if (ControllerIO.toggleGrabber) {
            subsystem.desiredClawOpen = !subsystem.desiredClawOpen
        }
        if (ControllerIO.toggleBrake) {
            subsystem.desiredBrake = !subsystem.desiredBrake
        }

        if (ControllerIO.armAlignUp) {
            subsystem.desiredArmAngle = if (subsystem.desiredTilt) {
                115.0
            } else {
                90.0
            }
        }

        if (ControllerIO.armAlignDown) {
            subsystem.desiredArmAngle = 40.0
        }

        if (ControllerIO.armAlignPickup) {
            subsystem.desiredArmAngle = 85.0
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
