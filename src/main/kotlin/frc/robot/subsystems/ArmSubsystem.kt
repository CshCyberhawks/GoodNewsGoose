package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.AnalogEncoder
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.PWM
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.MotorConstants
import frc.robot.util.IO

class ArmSubsystem : SubsystemBase() {
    var desiredTraversalExtended = false
    var desiredArmAngle = 0.0
        set(value) {
            armAnglePID.setpoint = desiredArmAngle
            field = value
        }

    private val armAngleMotor = CANSparkMax(MotorConstants.armAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val traversalMotor = TalonSRX(MotorConstants.traversalMotor)
    private val brakeSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.brakeSolenoid)
    private val grabSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.grabberSolenoid)

    private val armAnglePID = PIDController(0.1, 0.0, 0.0)

//    private val armAngleEncoder = Encoder(2, 3)
    private val armAngleEncoder = DutyCycleEncoder(2)
    private val traversalExtendedSwitch = DigitalInput(0)
    private val traversalRetractedSwitch = DigitalInput(1)

    init {
//        armAngleEncoder.distancePerPulse = 360.0 / 8192.0 // Set it to measure in degrees
        armAnglePID.enableContinuousInput(0.0, 360.0)
        armAnglePID.setTolerance(1.0)
        armAngleEncoder.distancePerRotation = 360.0
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
        return armAnglePID.atSetpoint() && ((desiredTraversalExtended && !traversalExtendedSwitch.get()) ||
                (!desiredTraversalExtended && !traversalRetractedSwitch.get()))
    }

    override fun periodic() {
//        SmartDashboard.putNumber("Desired Arm Angle", desiredArmAngle)
//        SmartDashboard.putBoolean("Desired Traversal", desiredTraversalExtended)
        SmartDashboard.putBoolean("Traversal Extended", !traversalExtendedSwitch.get())
        SmartDashboard.putBoolean("Traversal Retracted", !traversalRetractedSwitch.get())
//        SmartDashboard.putNumber("Arm Angle", getArmAngle())

//        SmartDashboard.putNumber(
//            "Arm Move", if (!armAnglePID.atSetpoint()) armAnglePID.calculate(getArmAngle()) else
//                0.0
//        )
//        armAngleMotor.set(
//            if (!armAnglePID.atSetpoint()) armAnglePID.calculate(getArmAngle()) else
//                0.0
//        )
        SmartDashboard.putBoolean("Grabber", IO.toggleGrabber)
        grabSolenoid.set(IO.toggleGrabber)
        brakeSolenoid.set(IO.toggleBrake)
        armAngleMotor.set(IO.controlArmAngle)
//        if (!armAnglePID.atSetpoint()) {
//            brakeSolenoid.set(false)
//            armAngleMotor.set(armAnglePID.calculate(getArmAngle()))
//        } else {
//            armAngleMotor.set(0.0)
//            brakeSolenoid.set(true)
//        }

//        val trav = if (!(!traversalExtendedSwitch.get() || !traversalRetractedSwitch.get())) {
//            if (desiredTraversalExtended) {
//                1.0
//            } else {
//                -1.0
//            }
//        } else {
//            0.0
//        }

//        val trav = if (desiredTraversalExtended && traversalExtendedSwitch.get()) {
//            -1.0
//        } else if (!desiredTraversalExtended && traversalRetractedSwitch.get()) {
//            1.0
//        } else {
//            0.0
//        }

//        SmartDashboard.putNumber("Traversal Output", trav)
//        traversalMotor[ControlMode.PercentOutput] = trav
        traversalMotor[ControlMode.PercentOutput] = IO.travManualControl

    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
