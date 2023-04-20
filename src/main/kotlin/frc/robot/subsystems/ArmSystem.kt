package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import cshcyberhawks.swolib.math.AngleCalculations
import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants
import frc.robot.constants.MotorConstants
import frc.robot.util.ControllerIO
import kotlin.math.abs

class ArmSystem : SubsystemBase() {
    private val tiltSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.tiltSolenoid)
    private val armAngleMotor = CANSparkMax(MotorConstants.armAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val extensionMotor = CANSparkMax(MotorConstants.extensionMotor, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val armAngleEncoder = DutyCycleEncoder(MotorConstants.armAngleEncoder)
    private val extensionEncoder = Encoder(4, 5)

    val extensionInBeamBreak = DigitalInput(MotorConstants.extensionInBeamBreak)
    val extensionMidBeamBreak = DigitalInput(MotorConstants.extensionMidBeamBreak)
    val extensionOutBeamBreak = DigitalInput(MotorConstants.extensionOutBeamBreak)

//    private val extensionExtendedSwitch = DigitalInput(MotorConstants.extensionExtendedSwitch)
//    private val extensionRetractedSwitch = DigitalInput(MotorConstants.extensionRetractedSwitch)
//
//    val extensionExtended
//        get() = extensionExtendedSwitch.get()
//
//    val extensionRetracted
//        get() = extensionRetractedSwitch.get()

    val armAngleDegrees
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360 - ArmConstants.armAngleOffset)

    val rawArmEncoder
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360)

    private var extensionPositionOffset = -1.0
    private val rawExtensionPosition
        get() = extensionEncoder.distance
    private val extensionPosition
        get() = rawExtensionPosition - extensionPositionOffset

    private val armAnglePID = PIDController(10.0, 0.0, 0.0)
    private var currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)
    private var desiredArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)

    private val armExtensionPID = PIDController(2.0, 0.0, 0.0)
    private var currentArmExtensionTrap = TrapezoidProfile.State(extensionPosition, 0.0)
    private var desiredArmExtensionTrap = TrapezoidProfile.State(extensionPosition, 0.0)
    private val armAngleTrapConstraints = TrapezoidProfile.Constraints(360.0, 45.0)
    private val armExtensionTrapConstraints = TrapezoidProfile.Constraints(3600.0, 3000.0)

//    private val armExtensionPID = PIDController(0.2, 0.0, 0.0)

    private var previousAngleTime = 0.0
    private var previousExtensionTime = 0.0
    //    private val armAnglePID = ProfiledPIDController(15.0, 0.0, 0.0, TrapezoidProfile.Constraints(360.0, 49.0))


    var desiredTilt = false
    var desiredArmAngle = armAngleDegrees
        set(value) {
//            armAnglePID.setGoal(value)
            armAnglePID.setpoint = value
            desiredArmAngleTrap = TrapezoidProfile.State(value, 0.0)
            currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)
            previousAngleTime = 0.0
//            armAnglePID.reset(armAngleDegrees)
            field = value
        }
    var desiredExtensionPosition = extensionPosition
        set(value) {
            armExtensionPID.setpoint = value
            desiredArmExtensionTrap = TrapezoidProfile.State(value, 0.0)
            currentArmAngleTrap = TrapezoidProfile.State(extensionPosition, 0.0)

            previousExtensionTime = 0.0
            field = value
        }
    var hitSetpoint = false
    var autoMode = false

    private fun setExtensionPosition(pos: Double) {
        // ep = raw - off
        // off = raw - ep
        extensionPositionOffset = rawExtensionPosition - pos
    }

    init {
        armAngleEncoder.distancePerRotation = 360.0
        armAnglePID.enableContinuousInput(0.0, 360.0)
        extensionEncoder.reset()
    }

    fun initialize() {
        desiredExtensionPosition = 0.0
        extensionPositionOffset = 0.0
    }

    fun run() {
        SmartDashboard.putNumber("Raw Extension Encoder", extensionEncoder.distance)
        SmartDashboard.putNumber("Extension Position", extensionPosition)

        // TODO: Measure these again and measure mid one properly
        if (extensionInBeamBreak.get()) {
            setExtensionPosition(ArmConstants.armExtensionIn)
//        } else if (extensionOutBeamBreak.get()) {
//            setExtensionPosition(ArmConstants.armExtensionOut)
//        } else if (extensionMidBeamBreak.get()) {
//            setExtensionPosition(ArmConstants.armExtensionMid)
        }

        if (desiredTilt && desiredExtensionPosition > ArmConstants.armExtensionMid) {
            desiredExtensionPosition = ArmConstants.armExtensionMid
        }

        SmartDashboard.putNumber("Extension Position", extensionPosition)
        SmartDashboard.putNumber("Raw Extension Position", rawExtensionPosition)

//        return;
        desiredArmAngle = MathUtil.clamp(desiredArmAngle, 35.0, if (desiredTilt) 135.0 else 100.0)
        if (desiredExtensionPosition < 0.0 || desiredExtensionPosition > ArmConstants.armExtensionOut) {
            desiredExtensionPosition = MathUtil.clamp(desiredExtensionPosition, 0.0, ArmConstants.armExtensionOut)
        }

        SmartDashboard.putBoolean("Extension In", extensionInBeamBreak.get())
        SmartDashboard.putBoolean("Extension Mid", extensionMidBeamBreak.get())
        SmartDashboard.putBoolean("Extension Out", extensionOutBeamBreak.get())


//        else if (extensionExtended) {
//            extensionPositionOffset = 3600 - rawExtensionPosition
//        }


        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)

        SmartDashboard.putNumber("Extension Setpoint", desiredExtensionPosition)

        val angleTimeNow = WPIUtilJNI.now() * 1.0e-6
        val angleTime: Double = if (previousAngleTime == 0.0) 0.0 else angleTimeNow - previousAngleTime

        val armPIDOutput = armAnglePID.calculate(armAngleDegrees) / 360

        val trapProfile = TrapezoidProfile(armAngleTrapConstraints, desiredArmAngleTrap, currentArmAngleTrap)

        val trapOutput = trapProfile.calculate(angleTime)

        val armOutput =
                if (armPIDOutput < 0) {
                    armPIDOutput - abs(trapOutput.velocity)
                } else {
                    armPIDOutput + abs(trapOutput.velocity)
                }

        currentArmAngleTrap = trapOutput

        if (extensionPositionOffset != -1.0) {
            val extensionManualControl = ControllerIO.extensionManualControl
            if (extensionManualControl != 0.0) {
                desiredExtensionPosition -= extensionManualControl
            } else {
                val extensionTimeNow = MiscCalculations.getCurrentTime()
                val extensionTime = if (previousExtensionTime == 0.0) 0.0 else extensionTimeNow - previousExtensionTime

                SmartDashboard.putNumber("Extension Trap Time", extensionTime)
                SmartDashboard.putNumber("Extension Prev Time", previousExtensionTime)
                SmartDashboard.putNumber("Extension Now Time", extensionTimeNow)

                val armExtensionPIDOutput = -(armExtensionPID.calculate(extensionPosition) / 360)

                SmartDashboard.putNumber("Extension PID", armExtensionPIDOutput)

                if (armExtensionPIDOutput < -0.1 || armExtensionPIDOutput > 0.1) {
                    val extensionTrap = TrapezoidProfile(armExtensionTrapConstraints, desiredArmExtensionTrap, currentArmExtensionTrap)

                    val extensionTrapOut = extensionTrap.calculate(extensionTime)
                    val extensionTrapOutput = extensionTrapOut.velocity / 3600

                    val extensionOutput = armExtensionPIDOutput * if (extensionTrapOutput != 0.0) abs(extensionTrapOutput) else 1.0

                    currentArmExtensionTrap = extensionTrapOut

                    SmartDashboard.putNumber("Extension Trap", extensionTrapOutput)
                    SmartDashboard.putNumber("Extension Output", extensionOutput)

                    extensionMotor.set(extensionOutput)

                    previousExtensionTime = extensionTimeNow
                } else if (desiredExtensionPosition == 0.0 && !extensionInBeamBreak.get()) {
                    extensionMotor.set(0.1)
                    setExtensionPosition(0.0)
                } else {
                    extensionMotor.set(0.01)
                }
            }
        } else {
            extensionMotor.set(0.1)
        }

//        SmartDashboard.putNumber("Traversal Velocity", extensionEncoder.velocity)

        armAngleMotor.set(armOutput)
//        armAngleMotor.set(ControllerIO.controlArmAngle)
        tiltSolenoid.set(desiredTilt)
//        extensionMotor.set(ControllerIO.extensionManualControl)

        SmartDashboard.putNumber("Desired Arm Angle", desiredArmAngle)

        previousAngleTime = angleTimeNow
    }

    fun kill() {
        armAngleMotor.set(0.0)
        extensionMotor.set(0.0)
    }

    fun isFinished(): Boolean {
        SmartDashboard.putBoolean("Extension Finished", abs(desiredExtensionPosition - extensionPosition) < 50.0)
        SmartDashboard.putBoolean("Arm Finished", abs(desiredArmAngle - armAngleDegrees) < 3.0)
        return abs(desiredExtensionPosition - extensionPosition) < 50.0 && abs(desiredArmAngle - armAngleDegrees) < 3.0
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
