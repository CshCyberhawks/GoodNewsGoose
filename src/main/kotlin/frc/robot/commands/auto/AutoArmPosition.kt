package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem
import java.util.*

/**
 * @property armSystem
 */
class AutoArmPosition(private val armSystem: ArmSystem, private val clawSystem: ClawSystem, private val armAngle: Double, private val extensionPosition: Double, private val tilt: Boolean, private val clawState: ClawState = ClawState.Idle) : CommandBase() {
    /**
     * Creates a new ExampleCommand.
     */
    init {
        // Use addRequirements() here to declare subsystem dependencies.
        //        addRequirements(armSystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        armSystem.desiredArmAngle = armAngle
        armSystem.desiredExtensionPosition = extensionPosition
        armSystem.desiredTilt = tilt
        armSystem.hitSetpoint = false
        clawSystem.clawState = clawState
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        armSystem.run()
        clawSystem.run()
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        armSystem.kill()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return armSystem.isFinished()
    }
}
