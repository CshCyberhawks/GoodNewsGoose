package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import frc.robot.constants.MotorConstants

enum class ClawState {
    Intaking,
    Spitting,
    Idle
}

class ClawSystem : SubsystemBase() {
    private val clawMotor = CANSparkMax(MotorConstants.clawMotor, CANSparkMaxLowLevel.MotorType.kBrushless)

    var clawState = ClawState.Idle
    fun run() {
        clawMotor.set(when (clawState) {
            ClawState.Intaking -> -0.4
            ClawState.Spitting -> 0.4
            ClawState.Idle -> -0.15
        })
    }

    fun kill() {
        clawState = ClawState.Idle
        run()
    }
}
