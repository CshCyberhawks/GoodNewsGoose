package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.math.AngleCalculations
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants
import frc.robot.constants.MotorConstants

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
    private val traversalExtendedSwitch = DigitalInput(MotorConstants.traversalExtendedSwitch)
    private val traversalRetractedSwitch = DigitalInput(MotorConstants.traversalRetractedSwitch)

    val armAngleDegrees
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360 - ArmConstants.armAngleOffset)
    private var traversalPosition = TraversalPosition.RETRACTED

    private var traversalOut = -0.25

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
    }

    fun run() {
        if (traversalExtendedSwitch.get()) {
            traversalPosition = TraversalPosition.EXTENDED
        } else if (traversalRetractedSwitch.get()) {
            traversalPosition = TraversalPosition.RETRACTED
        }

        // if (traversalPosition != TraversalPosition.UNKNOWN) {
        //     lastKnownTraversalPosition = traversalPosition
        // }

        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)
        val pidVal = -armAnglePID.calculate(armAngleDegrees) / 360
        SmartDashboard.putNumber("Arm PID Output", pidVal)

        armAngleMotor.set(pidVal)
        clawSolenoid.set(desiredClawOpen)
        tiltSolenoid.set(desiredTilt)
        SmartDashboard.putBoolean("Arm Out", traversalPosition == TraversalPosition.EXTENDED)
//        traversalMotor.set(desiredTraversalVelocity)
        SmartDashboard.putBoolean("At Pos", desiredTraversalPosition == traversalPosition)
        if (desiredTraversalPosition != traversalPosition) {
            traversalMotor.set(if (desiredTraversalPosition == TraversalPosition.EXTENDED) {
                traversalOut
            } else {
                -traversalOut
            })
        }
//        desiredTraversalVelocity = 0.0
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
