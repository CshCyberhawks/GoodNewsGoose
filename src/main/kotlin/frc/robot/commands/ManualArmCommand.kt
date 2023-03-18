package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.TraversalPosition
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
        subsystem.desiredTilt = ControllerIO.toggleTilt
        subsystem.desiredArmAngle += ControllerIO.controlArmAngle
        subsystem.desiredTraversalPosition = if (ControllerIO.traversalToggle) {
            TraversalPosition.EXTENDED
        } else {
            TraversalPosition.RETRACTED
        }
        subsystem.desiredClawOpen = ControllerIO.toggleGrabber

        subsystem.run()
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
