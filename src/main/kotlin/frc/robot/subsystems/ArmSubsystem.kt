package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ArmSubsystem(/*private val gyro: GenericGyro*/) : SubsystemBase() {
    var desiredTraversalExtended = false
    var desiredArmAngle = 0.0
        set(value) {
            armAnglePID.setpoint = desiredArmAngle
            field = value
        }

    private val armAngleMotor = VictorSPX(1)
    private val traversalMotor = VictorSPX(2)

    private val armAnglePID = PIDController(0.1, 0.0, 0.0)

    private val armAngleEncoder = Encoder(0, 1)
    private val traversalExtendedSwitch = DigitalInput(3)
    private val traversalRetractedSwitch = DigitalInput(4)

    init {
        armAngleEncoder.distancePerPulse = 360.0 / 8192.0 // Set it to measure in degrees
        armAnglePID.enableContinuousInput(0.0, 360.0)
        armAnglePID.setTolerance(1.0)
    }

    // Placeholder methods
    fun getArmAngle(): Double = armAngleEncoder.distance % 360

//    fun getRelativePositions(armAngle: Double = getArmAngle(), armTwist: Double = getArmTwist(), traversalLength: Double = getTraversalLength()): Vector3 {
//        val armAngleRadians = Math.toRadians(armAngle)
//
//        var x = (ArmMeasurements.armLength + traversalLength) * sin(armAngleRadians)
//        var y = (ArmMeasurements.armLength + traversalLength) * cos(armAngleRadians)
//
//        val armTwistRadians = Math.toRadians(armTwist)
//
//        val z = y
//        y = x * sin(armTwistRadians)
//        x *= cos(armTwistRadians)
//
////        val angleRadians = Math.toRadians(Math.toDegrees(atan2(y, x)) + gyro.getYaw())
////        val dist = sqrt(x * x + y * y)
//
////        return Vector3(dist * cos(angleRadians), dist * sin(angleRadians), z)
//        return Vector3(x, y, z)
//    }
//
//    fun setRelativeArmPosition(position: Vector3): Triple<Double, Double, Double> {
////        val angleTwistRadians = atan2(position.y, position.x) - Math.toRadians(gyro.getYaw())
////        val dist = sqrt(position.x * position.x + position.y * position.y)
//
////        var x = dist * cos(angleTwistRadians)
////        var y = dist * sin(angleTwistRadians)
//
//        val desiredArmTwistRadians = atan2(position.y, position.x)
//
//        val x = (position.x / cos(desiredArmTwistRadians) + position.y / sin(desiredArmTwistRadians)) / 2
//        val y = position.z
//
//        val desiredTraversalLength = sqrt(x * x + y * y) - ArmMeasurements.armLength
//        val desiredArmAngle = Math.toDegrees(atan2(y, x))
//
//        armAnglePID.setpoint = desiredArmAngle
//        armTwistPID.setpoint = Math.toDegrees(desiredArmTwistRadians)
//        traversalPID.setpoint = desiredTraversalLength
//
//        return Triple(desiredArmAngle, Math.toDegrees(desiredArmTwistRadians), desiredTraversalLength)
//    }

    fun isAtPos(): Boolean {
        return armAnglePID.atSetpoint() && ((desiredTraversalExtended && traversalExtendedSwitch.get()) || (!desiredTraversalExtended && traversalRetractedSwitch.get()))
    }

    override fun periodic() {
        SmartDashboard.putNumber("Desired Arm Angle", desiredArmAngle)
        SmartDashboard.putBoolean("Desired Traversal", desiredTraversalExtended)

        armAngleMotor[ControlMode.PercentOutput] = if (!armAnglePID.atSetpoint()) {
            armAnglePID.calculate(getArmAngle())
        } else {
             0.0
        }

        traversalMotor[ControlMode.PercentOutput] = if (!(traversalExtendedSwitch.get() || traversalRetractedSwitch.get())) {
            if (desiredTraversalExtended) {
                0.1
            } else {
                -0.1
            }
        } else {
            0.0
        }
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
