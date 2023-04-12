package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
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
//        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = subsystem.armAngleDegrees

//        setCurrentCommand(AutoArmPosition(subsystem, 90.0, 0.0, false, true))
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
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

        subsystem.desiredArmAngle += ControllerIO.controlArmAngle * 2

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

        if (ControllerIO.armAlignDown) {
            subsystem.desiredArmAngle = 35.0
        }

        if (ControllerIO.armAlignClosed) {
            subsystem.desiredArmAngle = 35.0
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
