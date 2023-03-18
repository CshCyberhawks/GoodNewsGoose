package frc.robot.subsystems

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

    private val armAngleEncoder = DutyCycleEncoder(MotorConstants.armAngleEncoder)
    private val extensionEncoder = DutyCycleEncoder(MotorConstants.extensionEncoder)
    private val extensionExtendedSwitch = DigitalInput(MotorConstants.extensionExtendedSwitch)
    private val extensionRetractedSwitch = DigitalInput(MotorConstants.extensionRetractedSwitch)

    private val extensionExtended
        get() = !extensionExtendedSwitch.get()

    private val extensionRetracted
        get() = !extensionRetractedSwitch.get()

    val armAngleDegrees
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360 - ArmConstants.armAngleOffset)
    private val extensionDistance
        get() = extensionEncoder.get()

    private var extensionPosition = ExtensionPosition.RETRACTED
    private var lastExtensionPosition = extensionPosition

    private var extensionFF = 0.6

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

    init {
        armAngleEncoder.distancePerRotation = 360.0
        armAnglePID.enableContinuousInput(0.0, 360.0)

        // This is sketchy
        extensionEncoder.positionOffset = extensionEncoder.absolutePosition
    }

    fun run() {
        desiredArmAngle = MathUtil.clamp(desiredArmAngle, 35.0, 115.0)

        extensionPosition = if (extensionExtended) {
            ExtensionPosition.EXTENDED
        } else if (extensionRetracted) {
            ExtensionPosition.RETRACTED
        } else {
            ExtensionPosition.UNKNOWN
        }

        if (extensionPosition != ExtensionPosition.UNKNOWN) {
            lastExtensionPosition = extensionPosition
        }

        SmartDashboard.putString("Traversal Position", extensionPosition.name)
        SmartDashboard.putString("Traversal Des Position", desiredExtensionPosition.name)
        SmartDashboard.putNumber("Traversal Angle", extensionDistance)

        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)
        val pidVal = -armAnglePID.calculate(armAngleDegrees) / 360
        SmartDashboard.putNumber("Arm PID Output", pidVal)

        SmartDashboard.putBoolean("Traversal Extended", extensionExtendedSwitch.get())
        SmartDashboard.putBoolean("Traversal Retracted", extensionRetractedSwitch.get())

        armAngleMotor.set(pidVal)
        clawSolenoid.set(desiredClawOpen)
        tiltSolenoid.set(desiredTilt)
        SmartDashboard.putBoolean("Arm Out", extensionPosition == ExtensionPosition.EXTENDED)
//        traversalMotor.set(desiredTraversalVelocity)
        SmartDashboard.putBoolean("At Pos", desiredExtensionPosition == extensionPosition)
        val extensionSetpoint = if (desiredExtensionPosition != lastExtensionPosition) {
            if (desiredExtensionPosition == ExtensionPosition.EXTENDED && extensionDistance > -1.0) {
                -extensionFF
            } else if (desiredExtensionPosition == ExtensionPosition.RETRACTED && extensionDistance < 0.1) {
                extensionFF
            } else {
                0.0
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
