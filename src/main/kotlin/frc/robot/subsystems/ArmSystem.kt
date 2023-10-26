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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.ArmConstants
import frc.robot.constants.MotorConstants
import frc.robot.util.ControllerIO
import kotlin.math.abs
import kotlin.math.sin

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

    private val armTab: ShuffleboardTab = Shuffleboard.getTab("Arm")

    val armAngleDegrees
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360 - ArmConstants.armAngleOffset)

    val rawArmEncoder
        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360)

    var extensionPositionOffset = -1.0
    private val rawExtensionPosition
        get() = extensionEncoder.distance
    val extensionPosition
        get() = rawExtensionPosition - extensionPositionOffset


    //p = 6, d = 0
    private val armAnglePID = PIDController(6.0, 0.0, 0.0)
    private var currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)
    private var desiredArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)

    private val armExtensionPID = PIDController(2.0, 0.0, 0.0)
    private var currentArmExtensionTrap = TrapezoidProfile.State(extensionPosition, 0.0)
    private var desiredArmExtensionTrap = TrapezoidProfile.State(extensionPosition, 0.0)
    //accel = 45
    private val armAngleTrapConstraints = TrapezoidProfile.Constraints(360.0, 45.0)
    private val armExtensionTrapConstraints = TrapezoidProfile.Constraints(3600.0, 3000.0)

//    private val armExtensionPID = PIDController(0.2, 0.0, 0.0)

    private var previousAngleTime = -1.0
    private var previousExtensionTime = 0.0
    //    private val armAnglePID = ProfiledPIDController(15.0, 0.0, 0.0, TrapezoidProfile.Constraints(360.0, 49.0))

    val extensionInShuffle = armTab.add("Extension In", false).entry
    val extensionPositionShuffle = armTab.add("Extension Position", 0.0).entry
    val extensionSetpointShuffle = armTab.add("Extension Setpoint", 0.0).entry
    val extensionTrapTimeShuffle = armTab.add("Extension Trap Time", 0.0).entry
    val extensionPIDShuffle = armTab.add("Extension PID", 0.0).entry
    val extensionTrapShuffle = armTab.add("Extension Trap", 0.0).entry
    val extensionOutputShuffle = armTab.add("Extension Output", 0.0).entry
    val extensionMotorOutputShuffle = armTab.add("Extension Motor Output", 0.0).entry
    val extensionMotorCurrentShuffle = armTab.add("Extension Motor Current", 0.0).entry
    val extensionMotorVelocityShuffle = armTab.add("Extension Motor Velocity", 0.0).entry
    val extensionMotorPositionShuffle = armTab.add("Extension Motor Position", 0.0).entry

    var desiredTilt = false
    var desiredArmAngle = armAngleDegrees
        set(value) {
//            armAnglePID.setGoal(value)
            armAnglePID.setpoint = value
            desiredArmAngleTrap = TrapezoidProfile.State(value, 0.0)
            currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)
//            previousAngleTime = 0.0
//            armAnglePID.reset(armAngleDegrees)
            field = value
        }
    var desiredExtensionPosition = 0.0
        set(value) {
            armExtensionPID.setpoint = value
            desiredArmExtensionTrap = TrapezoidProfile.State(value, 0.0)
            currentArmAngleTrap = TrapezoidProfile.State(extensionPosition, 0.0)

            previousExtensionTime = 0.0
            field = value
        }

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
        currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0);
    }

    fun run() {
        // TODO: Measure these again and measure mid one properly
        if (extensionInBeamBreak.get() && desiredExtensionPosition == 0.0) {
            setExtensionPosition(ArmConstants.armExtensionIn)
//        } else if (extensionOutBeamBreak.get()) {
//            setExtensionPosition(ArmConstants.armExtensionOut)
//        } else if (extensionMidBeamBreak.get()) {
//            setExtensionPosition(ArmConstants.armExtensionMid)
        }

//        SmartDashboard.putNumber("Extension Position", extensionPosition)
//        SmartDashboard.putNumber("Raw Extension Position", rawExtensionPosition)

        extensionPositionShuffle.setDouble(extensionPosition)

//        return;
        desiredArmAngle = MathUtil.clamp(desiredArmAngle, 35.0, if (desiredTilt) 130.0 else 115.0)
        desiredExtensionPosition = MathUtil.clamp(desiredExtensionPosition, 0.0, ArmConstants.armExtensionLimit)

        extensionMotorCurrentShuffle.setDouble(extensionMotor.outputCurrent)
        extensionMotorVelocityShuffle.setDouble(extensionMotor.encoder.velocity)
        extensionMotorPositionShuffle.setDouble(extensionMotor.encoder.position)

//        SmartDashboard.putBoolean("Extension In", extensionInBeamBreak.get())
        extensionInShuffle.setBoolean(extensionInBeamBreak.get())
//        SmartDashboard.putBoolean("Extension Mid", extensionMidBeamBreak.get())
//        SmartDashboard.putBoolean("Extension Out", extensionOutBeamBreak.get())

//        else if (extensionExtended) {
//            extensionPositionOffset = 3600 - rawExtensionPosition
//        }


        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
//        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)

//        SmartDashboard.putNumber("Extension Setpoint", desiredExtensionPosition)
        extensionSetpointShuffle.setDouble(desiredExtensionPosition)

        SmartDashboard.putNumber("Previous Angle Time", previousAngleTime)

        val angleTimeNow = WPIUtilJNI.now() * 1.0e-6
        SmartDashboard.putNumber("Angle Time Now", angleTimeNow)
        val angleTime: Double = if (previousAngleTime == -1.0) 0.0 else angleTimeNow - previousAngleTime


//        SmartDashboard.putNumber("Arm PID Output", armPIDOutput)

        val trapProfile = TrapezoidProfile(armAngleTrapConstraints, desiredArmAngleTrap, currentArmAngleTrap)

        val trapOutput = trapProfile.calculate(angleTime)

//        SmartDashboard.putNumber("Angle Time", angleTime)
//        SmartDashboard.putNumber("Arm Trap Desired", desiredArmAngleTrap.position)
//        SmartDashboard.putNumber("Arm Trap Current", currentArmAngleTrap.position)
//
//
//        SmartDashboard.putNumber("Arm Trap Output", trapOutput.velocity)

//        currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0);


        //r * f * sin(theta)
        //r = .7m f = 56 theta
        //gravity (nm) / max torque (nm)
        val gravityFF = ((.7) * (56) * sin(Math.toRadians(armAngleDegrees))) / 624
        SmartDashboard.putNumber("gravity ff", gravityFF)
//        var armOutput = armPIDOutput
        var armOutput = 0.0

        var ff = desiredArmAngle - armAngleDegrees


        SmartDashboard.putNumber("arm ff", ff)
        if (abs(ff) > 15) {
            armOutput += (ff / 60)
            SmartDashboard.putBoolean("ff engaged", true)
        }
        else {
            val armPIDOutput = armAnglePID.calculate(armAngleDegrees) / 360
            SmartDashboard.putNumber("arm pid", armPIDOutput)
            SmartDashboard.putBoolean("ff engaged", false)
            if (armAngleDegrees < 50 || desiredTilt) {
                armOutput += armPIDOutput + gravityFF
            }
            else {
                armOutput += armPIDOutput + gravityFF + (ff / 60)
            }

        }

        SmartDashboard.putNumber("arm output", armOutput);

//        SmartDashboard.putNumber("arm trap: ", abs(trapOutput.velocity))

//        if (armAngleDegrees > 135.0) {
//            armOutput = -0.15
//        }


        if (extensionPositionOffset != -1.0) {
            val extensionManualControl = ControllerIO.extensionManualControl
            if (extensionManualControl != 0.0) {
                desiredExtensionPosition -= extensionManualControl * 25
            }

            val extensionTimeNow = MiscCalculations.getCurrentTime()
            val extensionTime = if (previousExtensionTime == 0.0) 0.0 else extensionTimeNow - previousExtensionTime

//            SmartDashboard.putNumber("Extension Trap Time", extensionTime)
            extensionTrapTimeShuffle.setDouble(extensionTime)
//            SmartDashboard.putNumber("Extension Prev Time", previousExtensionTime)
//            SmartDashboard.putNumber("Extension Now Time", extensionTimeNow)

            val armExtensionPIDOutput = -(armExtensionPID.calculate(extensionPosition) / 360)

//            SmartDashboard.putNumber("Extension PID", armExtensionPIDOutput)
            extensionPIDShuffle.setDouble(armExtensionPIDOutput)

            if (extensionPosition <= 0.0 && !extensionInBeamBreak.get()) {
                extensionMotor.set(0.1)
            } else if (armExtensionPIDOutput < -0.1 || armExtensionPIDOutput > 0.1) {
                val extensionTrap = TrapezoidProfile(armExtensionTrapConstraints, desiredArmExtensionTrap, currentArmExtensionTrap)

                val extensionTrapOut = extensionTrap.calculate(extensionTime)
                val extensionTrapOutput = extensionTrapOut.velocity / 3600

                val extensionOutput = armExtensionPIDOutput * if (extensionTrapOutput != 0.0) abs(extensionTrapOutput) else 1.0

                currentArmExtensionTrap = extensionTrapOut

//                SmartDashboard.putNumber("Extension Trap", extensionTrapOutput)
//                SmartDashboard.putNumber("Extension Output", extensionOutput)

                extensionTrapShuffle.setDouble(extensionTrapOutput)
                extensionOutputShuffle.setDouble(extensionOutput)

                extensionMotor.set(extensionOutput)
                extensionMotorOutputShuffle.setDouble(extensionOutput)

                previousExtensionTime = extensionTimeNow
            } else if (desiredExtensionPosition == 0.0 && !extensionInBeamBreak.get()) {
                extensionMotor.set(0.1)
                extensionMotorOutputShuffle.setDouble(0.1)

            } else {
                extensionMotor.set(0.01)
                extensionMotorOutputShuffle.setDouble(0.01)
            }

//            if (desiredArmAngle <= 40.0 && abs(armAngleDegrees - desiredArmAngle) <= 4.0) {
//                armAngleMotor.set(0.0)
//            } else {
            armAngleMotor.set(armOutput)
//            }
        } else {
            armAngleMotor.set(0.0)
            extensionMotor.set(0.4)
            extensionMotorOutputShuffle.setDouble(0.1)
        }

//        SmartDashboard.putNumber("Traversal Velocity", extensionEncoder.velocity)

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

    fun resetPosition() {
        desiredTilt = false
        desiredArmAngle = ArmConstants.armInAngle
        desiredExtensionPosition = ArmConstants.armExtensionIn
    }

    fun isFinished(): Boolean {
        return abs(desiredExtensionPosition - extensionPosition) < 50.0 && abs(desiredArmAngle - armAngleDegrees) < 5.0
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
