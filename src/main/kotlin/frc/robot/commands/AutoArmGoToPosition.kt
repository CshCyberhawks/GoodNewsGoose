package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSubsystem
import frc.robot.subsystems.ExampleSubsystem

/**
 * @property subsystem
 */
class AutoArmGoToPosition(private val subsystem: ArmSubsystem, private val desiredArmAngle: Double, private val desiredTraversal: Boolean) : CommandBase() {
    /**
     * Creates a new ExampleCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = desiredArmAngle
        subsystem.desiredTraversalExtended = desiredTraversal
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {}

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return subsystem.isAtPos()
    }
}
