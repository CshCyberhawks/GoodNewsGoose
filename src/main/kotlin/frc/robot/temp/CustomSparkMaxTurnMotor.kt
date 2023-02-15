package frc.robot.temp

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.hardware.interfaces.GenericTurnMotor
import cshcyberhawks.swolib.math.AngleCalculations

class CustomSparkMaxTurnMotor(val deviceId: Int, val offset: Double) : GenericTurnMotor {
    val motor = CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless)
    val encoder = motor.encoder

    override fun get(): Double = getRaw() - offset

    override fun getRaw(): Double = AngleCalculations.wrapAroundAngles(encoder.position * 360)

    override fun setPercentOutput(percent: Double) {
        motor.set(percent)
    }
}