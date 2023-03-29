package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.math.AngleCalculations
import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants
import frc.robot.constants.MotorConstants
import frc.robot.util.ControllerIO

class ArmSystemGearBox : SubsystemBase() {
    private val tiltSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.tiltSolenoid)
    private val armAngleMotor = CANSparkMax(MotorConstants.armAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val extensionMotor = TalonFX(MotorConstants.extensionMotor)
    private val clawSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.grabberSolenoid)
    private val brakeSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.brakeSoleniod)

    private val armAngleEncoder = DutyCycleEncoder(MotorConstants.armAngleEncoder)
    private val extensionExtendedSwitch = DigitalInput(MotorConstants.extensionExtendedSwitch)
    private val extensionRetractedSwitch = DigitalInput(MotorConstants.extensionRetractedSwitch)

    val extensionExtended
        get() = extensionExtendedSwitch.get()

    val extensionRetracted
        get() = extensionRetractedSwitch.get()

    val armAngleDegrees
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360 - ArmConstants.armAngleOffset)

    val rawArmEncoder
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360)
    private val rawExtensionDistance
        get() = extensionMotor.selectedSensorPosition

    private val extensionVelocity
        get() = extensionMotor.selectedSensorVelocity

    private val armAnglePID = PIDController(10.0, 0.0, 0.0)
    private val armExtensionPID = PIDController(0.1, 0.0, 0.0)

    private var lastAngle = rawExtensionDistance

    private var extensionAngle = lastAngle

    var desiredTilt = false
    var desiredArmAngle = armAngleDegrees
        set(value) {
            armAnglePID.setpoint = value
//            armAnglePID.reset(armAngleDegrees)
            field = value
        }
    var desiredClawOpen = false
    var desiredBrake = false
    var usePID = true
    var desiredAngleSpeed = 0.0
    var desiredExtensionAngle = rawExtensionDistance

    init {
        armAngleEncoder.distancePerRotation = 360.0
        armAnglePID.enableContinuousInput(0.0, 360.0)

        extensionMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360)
    }

    fun run() {
        desiredArmAngle = MathUtil.clamp(desiredArmAngle, 40.0, 130.0)

        val currentAngle = rawExtensionDistance
        val currentVelocity = extensionVelocity
        var angleDifference = currentAngle - lastAngle
        if (angleDifference > 0 && currentVelocity < 0) {
            angleDifference -= 360
        } else if (angleDifference < 0 && currentVelocity > 0) {
            angleDifference += 360
        }

        extensionAngle += angleDifference

        SmartDashboard.putNumber("Traversal Position", extensionAngle)
        SmartDashboard.putNumber("Traversal Des Position", desiredArmAngle)
        SmartDashboard.putNumber("Traversal Angle", rawExtensionDistance)

        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)
        val armOutput = if (usePID) -armAnglePID.calculate(armAngleDegrees) / 360 else desiredAngleSpeed
        SmartDashboard.putNumber("Arm PID Output", armOutput)

        SmartDashboard.putBoolean("Traversal Extended", extensionExtendedSwitch.get())
        SmartDashboard.putBoolean("Traversal Retracted", extensionRetractedSwitch.get())

        armAngleMotor.set(armOutput)
        clawSolenoid.set(desiredClawOpen)
        tiltSolenoid.set(desiredTilt)
        brakeSolenoid.set(desiredBrake)
//        traversalMotor.set(desiredTraversalVelocity)
        val extensionSetpoint = armExtensionPID.calculate(extensionAngle, desiredExtensionAngle)
        SmartDashboard.putNumber("Traversal Set", extensionSetpoint)
        extensionMotor[ControlMode.PercentOutput] = (extensionSetpoint)
//        if (desiredTraversalPosition != traversalPosition) {
//            SmartDashboard.putNumber("Traversal Set", if (desiredTraversalPosition == TraversalPosition.EXTENDED) {
//                -traversalOut
//            } else {
//                traversalOut
//            })
//            traversalMotor.set(if (desiredTraversalPosition == TraversalPosition.EXTENDED) {
//                -traversalOut
//            } else {
//                traversalOut
//            })
//        } else {
//            SmartDashboard.putNumber("Traversal Set", 0.0)
//            traversalMotor.set(0.0)
//        }
//        val traversalManualControl = ControllerIO.traversalManualControl
//        SmartDashboard.putNumber("Trav Manual Control", traversalManualControl)
//        traversalMotor.set(traversalManualControl)
//        desiredTraversalVelocity = 0.0
    }

    fun isFinished(): Boolean {
        return desiredExtensionAngle == extensionAngle && MiscCalculations.calculateDeadzone(desiredArmAngle - armAngleDegrees, 3.0) == 0.0
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
