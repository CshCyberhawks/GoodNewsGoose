//package frc.robot.subsystems
//
//import com.revrobotics.CANSparkMax
//import com.revrobotics.CANSparkMaxLowLevel
//import cshcyberhawks.swolib.math.AngleCalculations
//import cshcyberhawks.swolib.math.MiscCalculations
//import edu.wpi.first.math.MathUtil
//import edu.wpi.first.math.controller.PIDController
//import edu.wpi.first.math.trajectory.TrapezoidProfile
//import edu.wpi.first.util.WPIUtilJNI
//import edu.wpi.first.wpilibj.DigitalInput
//import edu.wpi.first.wpilibj.DutyCycleEncoder
//import edu.wpi.first.wpilibj.Encoder
//import edu.wpi.first.wpilibj.PneumaticsModuleType
//import edu.wpi.first.wpilibj.Solenoid
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
//import edu.wpi.first.wpilibj2.command.SubsystemBase
//import frc.robot.constants.ArmConstants
//import frc.robot.constants.MotorConstants
//import frc.robot.util.ControllerIO
//import kotlin.math.abs
//
//enum class ExtensionPosition {
//    In,
//    Out,
//    Unknown
//}
//
//class ArmSystemLimit : SubsystemBase() {
//    private val tiltSolenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.tiltSolenoid)
//    private val armAngleMotor = CANSparkMax(MotorConstants.armAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushless)
//    private val extensionMotor = CANSparkMax(MotorConstants.extensionMotor, CANSparkMaxLowLevel.MotorType.kBrushless)
//
//    private val armAngleEncoder = DutyCycleEncoder(MotorConstants.armAngleEncoder)
//    private val extensionEncoder = Encoder(4, 5)
//
//    val extensionInBeamBreak = DigitalInput(MotorConstants.extensionInBeamBreak)
//    val extensionMidBeamBreak = DigitalInput(MotorConstants.extensionMidBeamBreak)
//
////    private val extensionExtendedSwitch = DigitalInput(MotorConstants.extensionExtendedSwitch)
////    private val extensionRetractedSwitch = DigitalInput(MotorConstants.extensionRetractedSwitch)
////
////    val extensionExtended
////        get() = extensionExtendedSwitch.get()
////
////    val extensionRetracted
////        get() = extensionRetractedSwitch.get()
//
//    private val armTab: ShuffleboardTab = Shuffleboard.getTab("Arm")
//
//    val armAngleDegrees
//        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360 - ArmConstants.armAngleOffset)
//
//    val rawArmEncoder
//        get() = AngleCalculations.wrapAroundAngles(armAngleEncoder.absolutePosition * 360)
//
//    private val armAnglePID = PIDController(10.0, 0.0, 0.0)
//    private var currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)
//    private var desiredArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)
//    private val armAngleTrapConstraints = TrapezoidProfile.Constraints(360.0, 45.0)
//
////    private val armExtensionPID = PIDController(0.2, 0.0, 0.0)
//
//    private var previousAngleTime = 0.0
//    //    private val armAnglePID = ProfiledPIDController(15.0, 0.0, 0.0, TrapezoidProfile.Constraints(360.0, 49.0))
//
//    val extensionInShuffle = armTab.add("Extension In", false).entry
//    val extensionPositionShuffle = armTab.add("Extension Position", 0.0).entry
//    val extensionSetpointShuffle = armTab.add("Extension Setpoint", 0.0).entry
//    val extensionTrapTimeShuffle = armTab.add("Extension Trap Time", 0.0).entry
//    val extensionPIDShuffle = armTab.add("Extension PID", 0.0).entry
//    val extensionTrapShuffle = armTab.add("Extension Trap", 0.0).entry
//    val extensionOutputShuffle = armTab.add("Extension Output", 0.0).entry
//    val extensionMotorOutputShuffle = armTab.add("Extension Motor Output", 0.0).entry
//    val extensionMotorCurrentShuffle = armTab.add("Extension Motor Current", 0.0).entry
//    val extensionMotorVelocityShuffle = armTab.add("Extension Motor Velocity", 0.0).entry
//    val extensionMotorPositionShuffle = armTab.add("Extension Motor Position", 0.0).entry
//
//    var desiredTilt = false
//    var desiredArmAngle = armAngleDegrees
//        set(value) {
////            armAnglePID.setGoal(value)
//            armAnglePID.setpoint = value
//            desiredArmAngleTrap = TrapezoidProfile.State(value, 0.0)
//            currentArmAngleTrap = TrapezoidProfile.State(armAngleDegrees, 0.0)
//            previousAngleTime = 0.0
////            armAnglePID.reset(armAngleDegrees)
//            field = value
//        }
//    var desiredExtensionPosition = ExtensionPosition.In
//
//    var extensionPosition = ExtensionPosition.Unknown
//
//    init {
//        armAngleEncoder.distancePerRotation = 360.0
//        armAnglePID.enableContinuousInput(0.0, 360.0)
//        extensionEncoder.reset()
//    }
//
//    fun initialize() {
//    }
//
//    fun run() {
//        // TODO: Measure these again and measure mid one properly
//        if (extensionInBeamBreak.get()) {
//            extensionPosition = ExtensionPosition.In
////        } else if (extensionOutBeamBreak.get()) {
////            setExtensionPosition(ArmConstants.armExtensionOut)
//        } else if (extensionMidBeamBreak.get()) {
//            extensionPosition = ExtensionPosition.Out
//        } else {
//            extensionPosition = ExtensionPosition.Unknown
//        }
//
//        val extensionAtPosition = extensionPosition == desiredExtensionPosition
//
//        SmartDashboard.putString("Extension Position", extensionPosition.name)
//
//        extensionPositionShuffle.setString(extensionPosition.name)
//
////        return;
//        desiredArmAngle = MathUtil.clamp(desiredArmAngle, 35.0, if (desiredTilt) 130.0 else 100.0)
//
//        extensionMotorCurrentShuffle.setDouble(extensionMotor.outputCurrent)
//        extensionMotorVelocityShuffle.setDouble(extensionMotor.encoder.velocity)
//        extensionMotorPositionShuffle.setDouble(extensionMotor.encoder.position)
//
//        SmartDashboard.putBoolean("Extension In", extensionInBeamBreak.get())
//        extensionInShuffle.setBoolean(extensionInBeamBreak.get())
//        SmartDashboard.putBoolean("Extension Mid", extensionMidBeamBreak.get())
//
////        else if (extensionExtended) {
////            extensionPositionOffset = 3600 - rawExtensionPosition
////        }
//
//
//        SmartDashboard.putNumber("Arm Angle", armAngleDegrees)
//        SmartDashboard.putNumber("Arm Setpoint", desiredArmAngle)
//
//        SmartDashboard.putString("Extension Setpoint", desiredExtensionPosition.name)
//        extensionSetpointShuffle.setString(desiredExtensionPosition.name)
//
//        val angleTimeNow = WPIUtilJNI.now() * 1.0e-6
//        val angleTime: Double = if (previousAngleTime == 0.0) 0.0 else angleTimeNow - previousAngleTime
//
//        val armPIDOutput = armAnglePID.calculate(armAngleDegrees) / 360
//
//        val trapProfile = TrapezoidProfile(armAngleTrapConstraints, desiredArmAngleTrap, currentArmAngleTrap)
//
//        val trapOutput = trapProfile.calculate(angleTime)
//
//        val armOutput =
//                if (armPIDOutput < 0) {
//                    armPIDOutput - abs(trapOutput.velocity)
//                } else {
//                    armPIDOutput + abs(trapOutput.velocity)
//                }
//
//        currentArmAngleTrap = trapOutput
//        if (extensionAtPosition) {
//            extensionMotor.set(0.0)
//        } else {
//            extensionMotor.set(0.5 * if (desiredExtensionPosition == ExtensionPosition.In) {
//                1
//            } else {
//                -1
//            })
//        }
////        SmartDashboard.putNumber("Traversal Velocity", extensionEncoder.velocity)
//
//        armAngleMotor.set(armOutput)
////        armAngleMotor.set(ControllerIO.controlArmAngle)
//        tiltSolenoid.set(desiredTilt)
////        extensionMotor.set(ControllerIO.extensionManualControl)
//
//        SmartDashboard.putNumber("Desired Arm Angle", desiredArmAngle)
//
//        previousAngleTime = angleTimeNow
//    }
//
//    fun kill() {
//        armAngleMotor.set(0.0)
//        extensionMotor.set(0.0)
//    }
//
//    fun isFinished(): Boolean {
////        return abs(desiredExtensionPosition - extensionPosition) < 50.0 && abs(desiredArmAngle - armAngleDegrees) < 3.0
//        return false
//    }
//
//    override fun simulationPeriodic() {
//        // This method will be called once per scheduler run during simulation
//    }
//}
