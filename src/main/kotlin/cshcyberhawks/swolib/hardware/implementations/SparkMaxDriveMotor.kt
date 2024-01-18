package cshcyberhawks.swolib.hardware.implementations

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.hardware.enums.MotorNeutralMode
import cshcyberhawks.swolib.hardware.interfaces.GenericDriveMotor

class SparkMaxDriveMotor(deviceId: Int, canBus: String = "") : GenericDriveMotor {
    private val motor = CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless)

    override fun setInverted(inverted: Boolean) {
        motor.inverted = inverted
    }

    override fun setNeutralMode(inputMode: MotorNeutralMode) {
        val mode = when (inputMode) {
            MotorNeutralMode.Coast -> IdleMode.kCoast
            MotorNeutralMode.Brake -> IdleMode.kBrake
        }
        motor.setNeutralMode(mode)
    }

    override fun getVelocity(): Double = motor.selectedSensorVelocity / 204.8

    override fun setPercentOutput(percent: Double) {
        motor[ControlMode.PercentOutput] = percent
    }
}