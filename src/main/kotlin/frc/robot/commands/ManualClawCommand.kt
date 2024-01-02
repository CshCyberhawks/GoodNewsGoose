package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem
import frc.robot.util.ControllerIO

/**
 * @property subsystem
 */
class ManualClawCommand() : CommandBase() {
    /**
     * Creates a new ExampleCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (ControllerIO.clawSpit) {
            ClawSystem.clawState = ClawState.Spitting
        } else if (ControllerIO.spinClaw) {
            ClawSystem.clawState = ClawState.Intaking
        }

        ClawSystem.run()
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}