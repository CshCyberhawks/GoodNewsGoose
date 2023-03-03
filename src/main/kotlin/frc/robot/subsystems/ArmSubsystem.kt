package frc.robot.subsystems

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.Vector3
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmMeasurements
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class ArmSubsystem(private val gyro: GenericGyro) : SubsystemBase() {
    private val armAnglePID = PIDController(0.1, 0.0, 0.0)
    private val traversalPID = PIDController(0.1, 0.0, 0.0)
    private val armTwistPID = PIDController(0.1, 0.0, 0.0)

    // Placeholder methods
    private fun getArmAngle(): Double = 0.0
    private fun getTraversalLength(): Double = 0.0
    private fun getArmTwist(): Double = 0.0

    fun getRelativePositions(): Vector3 {
        val armAngleRadians = Math.toRadians(getArmAngle())

        var x = (ArmMeasurements.armLength + getTraversalLength()) * sin(armAngleRadians)
        var y = (ArmMeasurements.armLength + getTraversalLength()) * cos(armAngleRadians)

        val armTwistRadians = Math.toRadians(getArmTwist())

        val z = y
        y = x * sin(armTwistRadians)
        x *= cos(armTwistRadians)

        val angleRadians = Math.toRadians(Math.toDegrees(atan2(y, x)) + gyro.getYaw())
        val dist = sqrt(x * x + y * y)


        return Vector3(dist * cos(angleRadians), dist * sin(angleRadians), z)
    }

    fun setRelativeArmPosition(position: Vector3) {
        val angleTwistRadians = atan2(position.y, position.x) - Math.toRadians(gyro.getYaw())
        val dist = sqrt(position.x * position.x + position.y * position.y)

        var x = dist * cos(angleTwistRadians)
        var y = dist * sin(angleTwistRadians)

        val desiredArmTwist = atan2(y, x)

        x = (x / cos(desiredArmTwist) + y / sin(desiredArmTwist)) / 2
        y = position.z

        val desiredTraversalLength = sqrt(x * x + y * y)
        val desiredArmAngle = atan2(y, x)

        armAnglePID.setpoint = desiredArmAngle
        armTwistPID.setpoint = desiredArmTwist
        traversalPID.setpoint = desiredTraversalLength
    }

    override fun periodic() {
        // This method will be called once per scheduler run
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
