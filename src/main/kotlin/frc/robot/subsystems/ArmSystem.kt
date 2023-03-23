package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.hardware.implementations.VelocityDutyCycleEncoder
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

enum class ExtensionPosition {
    EXTENDED,
    RETRACTED,
    UNKNOWN
}

class ArmSystem : SubsystemBase() {
    private val tiltSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.tiltSolenoid)
    private val armAngleMotor = CANSparkMax(MotorConstants.armAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val extensionMotor = CANSparkMax(MotorConstants.extensionMotor, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val clawSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.grabberSolenoid)
    private val brakeSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.brakeSoleniod)

    private val armAngleEncoder = DutyCycleEncoder(MotorConstants.armAngleEncoder)
    private val extensionEncoder = VelocityDutyCycleEncoder(MotorConstants.extensionEncoder)
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
    private val extensionDistance
        get() = extensionEncoder.get()

    private var extensionPosition = ExtensionPosition.RETRACTED

    private val armAnglePID = PIDController(10.0, 0.0, 0.0)

    var desiredTilt = false
    var desiredArmAngle = armAngleDegrees
        set(value) {
            armAnglePID.setpoint = value
//            armAnglePID.reset(armAngleDegrees)
            field = value
        }
    var desiredExtensionPosition = ExtensionPosition.RETRACTED
    var desiredClawOpen = false
    var desiredBrake = false
    var usePID = true
    var desiredAngleSpeed = 0.0

    init {
        armAngleEncoder.distancePerRotation = 360.0
        armAnglePID.enableContinuousInput(0.0, 360.0)

        // This is sketchy
        extensionEncoder.positionOffset = extensionEncoder.absolutePosition
    }

    fun run() {
        desiredArmAngle = MathUtil.clamp(desiredArmAngle, 40.0, 130.0)


        extensionPosition = if (extensionExtended) {
            ExtensionPosition.EXTENDED
        } else if (extensionRetracted) {
            ExtensionPosition.RETRACTED
        } else {
            ExtensionPosition.UNKNOWN
        }

        SmartDashboard.putString("Traversal Position", extensionPosition.name)
        SmartDashboard.putString("Traversal Des Position", desiredExtensionPosition.name)
        SmartDashboard.putNumber("Traversal Angle", extensionDistance)

        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)
        val armOutput = if (usePID) -armAnglePID.calculate(armAngleDegrees) / 360 else desiredAngleSpeed
        SmartDashboard.putNumber("Arm PID Output", armOutput)

        SmartDashboard.putBoolean("Traversal Extended", extensionExtendedSwitch.get())
        SmartDashboard.putBoolean("Traversal Retracted", extensionRetractedSwitch.get())

        SmartDashboard.putNumber("Traversal Velocity", extensionEncoder.velocity)

        armAngleMotor.set(armOutput)
        clawSolenoid.set(desiredClawOpen)
        tiltSolenoid.set(desiredTilt)
        brakeSolenoid.set(desiredBrake)
        SmartDashboard.putBoolean("Arm Out", extensionPosition == ExtensionPosition.EXTENDED)
//        traversalMotor.set(desiredTraversalVelocity)
        SmartDashboard.putBoolean("At Pos", desiredExtensionPosition == extensionPosition)
        val manualTraversalSetpoint = MiscCalculations.calculateDeadzone(ControllerIO.extensionManualControl, 0.05)
        val extensionSetpoint = if (manualTraversalSetpoint != 0.0) {
            // TODO: Less sketchy
            desiredExtensionPosition = ExtensionPosition.UNKNOWN
            manualTraversalSetpoint
        } else if (desiredExtensionPosition != extensionPosition) {
            when (desiredExtensionPosition) {
                ExtensionPosition.EXTENDED -> -1.0
                ExtensionPosition.RETRACTED -> 1.0
                else -> 0.0
            }
        } else {
            0.0
        }
        SmartDashboard.putNumber("Traversal Set", extensionSetpoint)
        extensionMotor.set(extensionSetpoint)
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
        return desiredExtensionPosition == extensionPosition && MiscCalculations.calculateDeadzone(desiredArmAngle - armAngleDegrees, 3.0) == 0.0
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
