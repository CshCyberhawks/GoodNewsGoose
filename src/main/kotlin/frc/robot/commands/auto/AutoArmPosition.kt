package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem
import java.util.*

/**
 * @property armSystem
 */
class AutoArmPosition(private val armSystem: ArmSystem, private val clawSystem: ClawSystem, armAngleInput: Double? = null, extensionPositionInput: Double? = null, tiltInput: Boolean? = null, clawStateInput: ClawState? = null) : CommandBase() {
    companion object {
        var armAngle = 35.0
        var extensionPosition = 0.0
        var tilt = false
        var clawState = ClawState.Idle
    }

    init {
        if (armAngleInput != null) {
            armAngle = armAngleInput
        }
        if (extensionPositionInput != null) {
            extensionPosition = extensionPositionInput
        }
        if (tiltInput != null) {
            tilt = tiltInput
        }
        if (clawStateInput != null) {
            clawState = clawStateInput
        }
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
