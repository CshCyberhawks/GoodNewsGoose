package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.math.AngleCalculations
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

enum class TraversalPosition {
    EXTENDED,
    RETRACTED,
    UNKNOWN
}

class ArmSystem : SubsystemBase() {
    private val tiltSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.tiltSolenoid)
    private val armAngleMotor = CANSparkMax(MotorConstants.armAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val traversalMotor = CANSparkMax(MotorConstants.traversalMotor, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val clawSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.grabberSolenoid)

    private val armAngleEncoder = DutyCycleEncoder(MotorConstants.armAngleEncoder)
    private val traversalEncoder = DutyCycleEncoder(MotorConstants.traversalEncoder)
    private val traversalExtendedSwitch = DigitalInput(MotorConstants.traversalExtendedSwitch)
    private val traversalRetractedSwitch = DigitalInput(MotorConstants.traversalRetractedSwitch)

    private val traversalExtended
        get() = !traversalExtendedSwitch.get()

    private val traversalRetracted
        get() = !traversalRetractedSwitch.get()

    val armAngleDegrees
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360 - ArmConstants.armAngleOffset)
    val traversalDistance
        get() = traversalEncoder.get()

    private var traversalPosition = TraversalPosition.RETRACTED
    private var lastTraversalPosition = traversalPosition

    private var traversalOut = 0.6

    private val armAnglePID = PIDController(10.0, 0.0, 0.0)

    var desiredTilt = false
    var desiredArmAngle = armAngleDegrees
        set(value) {
            armAnglePID.setpoint = value
//            armAnglePID.reset(armAngleDegrees)
            field = value
        }
    var desiredTraversalPosition = TraversalPosition.RETRACTED
    var desiredClawOpen = false

    init {
        armAngleEncoder.distancePerRotation = 360.0
        armAnglePID.enableContinuousInput(0.0, 360.0)

        // This is sketchy
        traversalEncoder.positionOffset = traversalEncoder.absolutePosition
    }

    fun run() {
        desiredArmAngle = MathUtil.clamp(desiredArmAngle, 35.0, 115.0)

        traversalPosition = if (traversalExtended) {
            TraversalPosition.EXTENDED
        } else if (traversalRetracted) {
            TraversalPosition.RETRACTED
        } else {
            TraversalPosition.UNKNOWN
        }

        if (traversalPosition != TraversalPosition.UNKNOWN) {
            lastTraversalPosition = traversalPosition
        }

        SmartDashboard.putString("Traversal Position", traversalPosition.name)
        SmartDashboard.putString("Traversal Des Position", desiredTraversalPosition.name)
        SmartDashboard.putNumber("Traversal Angle", traversalDistance)

        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)
        val pidVal = -armAnglePID.calculate(armAngleDegrees) / 360
        SmartDashboard.putNumber("Arm PID Output", pidVal)

        SmartDashboard.putBoolean("Traversal Extended", traversalExtendedSwitch.get())
        SmartDashboard.putBoolean("Traversal Retracted", traversalRetractedSwitch.get())

        armAngleMotor.set(pidVal)
        clawSolenoid.set(desiredClawOpen)
        tiltSolenoid.set(desiredTilt)
        SmartDashboard.putBoolean("Arm Out", traversalPosition == TraversalPosition.EXTENDED)
//        traversalMotor.set(desiredTraversalVelocity)
        SmartDashboard.putBoolean("At Pos", desiredTraversalPosition == traversalPosition)
        val traversalManualControl = ControllerIO.traversalManualControl
//        if (traversalManualControl != 0.0) {
//        SmartDashboard.putNumber("Traversal Set", traversalManualControl)
//        traversalMotor.set(traversalManualControl)
//        } else
        val traversalSetpoint = if (desiredTraversalPosition != lastTraversalPosition) {
            if (desiredTraversalPosition == TraversalPosition.EXTENDED && traversalDistance > -1.0) {
                -traversalOut
            } else if (desiredTraversalPosition == TraversalPosition.RETRACTED && traversalDistance < 0.1) {
                traversalOut
            } else {
                0.0
            }
        } else {
            0.0
        }
        SmartDashboard.putNumber("Traversal Set", traversalSetpoint)
        traversalMotor.set(traversalSetpoint)
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

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
