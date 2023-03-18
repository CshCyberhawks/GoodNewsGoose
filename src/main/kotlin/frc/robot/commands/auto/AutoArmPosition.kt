package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.TraversalPosition

/**
 * @property subsystem
 */
class AutoArmPosition(private val subsystem: ArmSystem, private val armAngle: Double, private val traversalPosition: TraversalPosition, private val tilt: Boolean) : CommandBase() {
    /**
     * Creates a new ExampleCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = armAngle
        subsystem.desiredTraversalPosition = traversalPosition
        subsystem.desiredTilt = tilt
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        subsystem.run()
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return subsystem.isFinished()
    }
}
