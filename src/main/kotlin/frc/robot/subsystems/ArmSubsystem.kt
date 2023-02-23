package frc.robot.subsystems

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.Vector3
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmMeasurements
import kotlin.math.*

class ArmSubsystem(private val gyro: GenericGyro) : SubsystemBase() {
    private val armAnglePID = PIDController(0.1, 0.0, 0.0)
    private val traversalPID = PIDController(0.1, 0.0, 0.0)

    // Placeholder methods
    private fun getArmAngle(): Double = 0.0
    private fun getTraversalLength(): Double = 0.0
    private fun getArmTwist(): Double = 0.0

    fun getRelativePosition(): Vector3 {
        var x = ArmMeasurements.armLength * sin(Math.toRadians(getArmAngle())) + getTraversalLength()
        var y = ArmMeasurements.armHeight - ArmMeasurements.armLength * cos(Math.toRadians(getArmAngle()))

        val z = y
        y = x * sin(Math.toRadians(getArmTwist()))
        x *= cos(Math.toRadians(getArmTwist()))

        val angleRadians = Math.toRadians(Math.toDegrees(atan2(y, x)) + gyro.getYaw())
        val dist = sqrt(x * x + y * y)

        return Vector3(dist * cos(angleRadians), dist * sin(angleRadians), z)
    }

    fun setRelativeArmPosition(position: Vector3) {
        val angleRadians = Math.toRadians(Math.toDegrees(atan2(position.y, position.x)) - gyro.getYaw())
        val dist = sqrt(position.x * position.x + position.y * position.y)
        val armAngleRadians = Math.toRadians(getArmTwist())

        val x = (dist * cos(angleRadians) / cos(armAngleRadians) + dist * sin(angleRadians) / sin(armAngleRadians)) / 2
        val y = position.z

        val desiredArmAngle = Math.toDegrees(acos((ArmMeasurements.armHeight - y) / ArmMeasurements.armLength))
        val desiredTraversalLength = x - ArmMeasurements.armLength * sin(Math.toRadians(desiredArmAngle))

        armAnglePID.setpoint = desiredArmAngle
        traversalPID.setpoint = desiredTraversalLength
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
