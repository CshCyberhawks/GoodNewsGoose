package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.constants.MotorConstants

enum class ClawState {
    Intaking,
    Spitting,
    Idle
}

class ClawSystem : SubsystemBase() {
    private val clawMotor = CANSparkMax(MotorConstants.clawMotor, CANSparkMaxLowLevel.MotorType.kBrushless)

    val intakeBeamBreak = DigitalInput(MotorConstants.intakeBeamBreak)

    private var breakTime = -1.0

//    private var wheelSpeedShuffle = Shuffleboard.getTab("Driver").add("Wheel Speed", -.9).entry

    var clawState = ClawState.Idle
    fun run() {
        if (clawState == ClawState.Intaking && !intakeBeamBreak.get() && breakTime == -1.0) {
            breakTime = MiscCalculations.getCurrentTime()
        }

        if (clawState != ClawState.Intaking) {
            breakTime = -1.0
        }

        if (breakTime != -1.0 && MiscCalculations.getCurrentTime() >= breakTime + 0.2) {
            clawState = ClawState.Idle
        }


        SmartDashboard.putString("Claw State", clawState.name)
        SmartDashboard.putBoolean("Claw Break", intakeBeamBreak.get())

        clawMotor.set(when (clawState) {
            ClawState.Intaking -> 1.0
            ClawState.Spitting -> -1.0
            ClawState.Idle -> 0.05
        })

        SmartDashboard.putNumber("Current", clawMotor.outputCurrent)
    }

    fun kill() {
        clawState = ClawState.Idle
        run()
    }
}