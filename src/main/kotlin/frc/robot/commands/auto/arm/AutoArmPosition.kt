package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ExtensionPosition

/**
 * @property subsystem
 */
class AutoArmPosition(private val subsystem: ArmSystem, private val armAngle: Double, private val extensionPosition: ExtensionPosition, private val tilt: Boolean, private val clawOpen: Boolean) : CommandBase() {
    /**
     * Creates a new ExampleCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.usePID = true
        subsystem.desiredArmAngle = armAngle
        subsystem.desiredExtensionPosition = extensionPosition
        subsystem.desiredTilt = tilt
        subsystem.desiredClawOpen = clawOpen
        subsystem.hitSetpoint = false
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
