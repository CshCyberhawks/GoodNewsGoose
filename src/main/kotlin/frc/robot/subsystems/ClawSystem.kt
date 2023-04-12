package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import frc.robot.constants.MotorConstants

class ClawSystem : SubsystemBase() {
    private val clawMotor = CANSparkMax(MotorConstants.clawMotor, CANSparkMaxLowLevel.MotorType.kBrushless)

    var clawSpinning = false
    var clawSpitting = false

    fun run() {
        clawMotor.set(if (clawSpitting) {
            0.4
        } else if (clawSpinning) {
            -0.4
        } else {
            -0.15
        })
    }
}
