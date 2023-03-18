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
        if (ControllerIO.toggleTilt) {
            subsystem.desiredTilt = !subsystem.desiredTilt
        }
        subsystem.desiredArmAngle += ControllerIO.controlArmAngle
        if (ControllerIO.traversalToggle) {
            if (subsystem.desiredTraversalPosition == TraversalPosition.EXTENDED) {
                subsystem.desiredTraversalPosition = TraversalPosition.RETRACTED
            } else if (subsystem.desiredTraversalPosition == TraversalPosition.RETRACTED) {
                subsystem.desiredTraversalPosition = TraversalPosition.EXTENDED
            }
        }
        if (ControllerIO.toggleGrabber) {
            subsystem.desiredClawOpen = !subsystem.desiredClawOpen
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
