package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem

/**
 * @property subsystem
 */
class AutoArmPosition(private val subsystem: ArmSystem, private val armAngle: Double, private val extensionPosition: Double, private val tilt: Boolean, private val clawOpen: Boolean) : CommandBase() {
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
        subsystem.clawSpinning = clawOpen
        subsystem.hitSetpoint = false
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        subsystem.run()
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        subsystem.kill()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return subsystem.isFinished()
    }
}
