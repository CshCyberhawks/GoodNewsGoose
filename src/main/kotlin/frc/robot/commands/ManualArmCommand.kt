package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSubsystem
import frc.robot.util.IO

/**
 * @property subsystem
 */
class ManualArmCommand(private val subsystem: ArmSubsystem) : CommandBase() {
    var lastTraversal = false

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        subsystem.desiredArmAngle += IO.controlArmAngle
        subsystem.desiredArmAngle %= 360

        SmartDashboard.putBoolean("IO Toggle", IO.toggleTraversal)
        if (IO.toggleTraversal && !lastTraversal) {
            subsystem.desiredTraversalExtended = !subsystem.desiredTraversalExtended
        }

        lastTraversal = IO.toggleTraversal
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
