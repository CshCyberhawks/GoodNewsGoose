package frc.robot

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.field2d.Field2d
import cshcyberhawks.swolib.hardware.implementations.Pigeon2Gyro
import cshcyberhawks.swolib.hardware.implementations.SparkMaxTurnMotor
import cshcyberhawks.swolib.hardware.implementations.TalonFXDriveMotor
import cshcyberhawks.swolib.limelight.LedMode
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector3
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import cshcyberhawks.swolib.swerve.SwerveOdometry
import cshcyberhawks.swolib.swerve.SwerveWheel
import cshcyberhawks.swolib.swerve.configurations.FourWheelAngleConfiguration
import cshcyberhawks.swolib.swerve.configurations.FourWheelSpeedConfiguration
import cshcyberhawks.swolib.swerve.configurations.FourWheelSwerveConfiguration
import cshcyberhawks.swolib.swerve.configurations.SwerveModuleConfiguration
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.HttpCamera
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.*
import frc.robot.constants.MotorConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawSystem
import java.util.*
import java.util.function.Supplier


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    companion object {
        var pipIndex = 2
    }

    private val driverTab: ShuffleboardTab = Shuffleboard.getTab("Driver")

    private val autoChooser = SendableChooser<AutoSequenceType>()

    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null
    private val swerveConfiguration: SwerveModuleConfiguration =
            SwerveModuleConfiguration(4.0, 0.0505, 7.0)
    private val drivePIDBackLeft = PIDController(0.01, 0.0, 0.0)
    private val turnPIDBackLeft = PIDController(.012, 0.0, 0.0002)

    private val drivePIDBackRight = PIDController(0.01, 0.0, 0.0)
    private val turnPIDBackRight = PIDController(.012, 0.0, 0.0002)

    private val drivePIDFrontLeft = PIDController(0.01, 0.0, 0.0)
    private val turnPIDFrontLeft = PIDController(.012, 0.0, 0.0002)

    private val drivePIDFrontRight = PIDController(0.01, 0.0, 0.0)
    private val turnPIDFrontRight = PIDController(.012, 0.0, 0.0002)

    //
    private val limelightLeft = Limelight("limelight-left", 0.74, 0.0, fiducialPipeline = 1)
    private val limelightRight = Limelight("limelight-right", 0.74, 0.0, fiducialPipeline = 1)

    private var backLeft: SwerveWheel =
            SwerveWheel(
                    TalonFXDriveMotor(MotorConstants.backLeftDriveMotor),
                    SparkMaxTurnMotor(
                            MotorConstants.backLeftTurnMotor,
                            MotorConstants.backLeftEncoder,
                            MotorConstants.turnEncoderOffsets[MotorConstants.backLeftEncoder - 10]
                    ),
                    drivePIDBackLeft,
                    turnPIDBackLeft,
                    swerveConfiguration
            )
    private var backRight: SwerveWheel =
            SwerveWheel(
                    TalonFXDriveMotor(MotorConstants.backRightDriveMotor),
                    SparkMaxTurnMotor(
                            MotorConstants.backRightTurnMotor,
                            MotorConstants.backRightEncoder,
                            MotorConstants.turnEncoderOffsets[MotorConstants.backRightEncoder - 10]
                    ),
                    drivePIDBackRight,
                    turnPIDBackRight,
                    swerveConfiguration
            )
    private var frontLeft: SwerveWheel =
            SwerveWheel(
                    TalonFXDriveMotor(MotorConstants.frontLeftDriveMotor),
                    SparkMaxTurnMotor(
                            MotorConstants.frontLeftTurnMotor,
                            MotorConstants.frontLeftEncoder,
                            MotorConstants.turnEncoderOffsets[MotorConstants.frontLeftEncoder - 10]
                    ),
                    drivePIDFrontLeft,
                    turnPIDFrontLeft,
                    swerveConfiguration
            )
    private var frontRight: SwerveWheel =
            SwerveWheel(
                    TalonFXDriveMotor(MotorConstants.frontRightDriveMotor),
                    SparkMaxTurnMotor(
                            MotorConstants.frontRightTurnMotor,
                            MotorConstants.frontRightEncoder,
                            MotorConstants.turnEncoderOffsets[MotorConstants.frontRightEncoder - 10]
                    ),
                    drivePIDFrontRight,
                    turnPIDFrontRight,
                    swerveConfiguration
            )

    //
    val gyro = Pigeon2Gyro(30)

    //
    private val swerveDriveTrain =
            SwerveDriveTrain(
                    FourWheelSwerveConfiguration(
                            frontRight,
                            frontLeft,
                            backRight,
                            backLeft,
                            angleConfiguration =
                            FourWheelAngleConfiguration(131.6, -131.6, 48.4, -48.4),
                            speedConfiguration = FourWheelSpeedConfiguration(.65, .65, .65, .65)
                    ),
                    gyro
            )

    private val field2d = Field2d()

    private val swo =
            SwerveOdometry(
                    swerveDriveTrain,
                    gyro,
                    1.0,
                    Vector3(0.0, 0.0, 0.0),
                    arrayOf(limelightLeft),
                    debugLogging = true,
                    field2d = Optional.of(field2d)
            )

    // val autoTrapConstraints = TrapezoidProfile.Constraints(4.0, 1.0)
    private val autoTrapConstraints = TrapezoidProfile.Constraints(5.0, 3.0)
    private val twistTrapConstraints = TrapezoidProfile.Constraints(90.0, 20.0)

    private val autoPIDX = ProfiledPIDController(1.0, 0.0, 0.01, autoTrapConstraints)
    private val autoPIDY = ProfiledPIDController(1.0, 0.0, 0.01, autoTrapConstraints)
    private val twistPID = PIDController(0.1, 0.0, 0.00)

    private val auto =
            SwerveAuto(
                    autoPIDX,
                    autoPIDY,
                    twistPID,
                    twistTrapConstraints,
                    // TrapezoidProfile.Constraints(4.0, 1.5),
                    5.0, // TODO: Tune PIDs so this can be smaller
                    0.05,
                    swo,
                    swerveDriveTrain,
                    gyro,
                    true,
                    Optional.of(field2d)
            )

    private var teleopSwerveCommand =
            TeleopSwerveCommand(
                    swerveDriveTrain,
                    auto,
                    gyro,
                    driverTab,
                    arrayOf(limelightRight, limelightLeft)
            )

    private val autoPathManager = AutoPathManager(auto, gyro)

    private var armSystem = ArmSystem()
    private val clawSystem = ClawSystem()

    var autoCommand: CommandBase? = null

    private var teleopArmCommand = ManualArmCommand(armSystem)
    private var teleopClawCommand = ManualClawCommand(clawSystem)

    private val odometryResetLLShuffle =
            driverTab.add("Reset Odometry With Limelight", true).getEntry()

    lateinit var llCam: HttpCamera

//    private val odometryFilterX: MedianFilter = MedianFilter(10)
//    private val odometryFilterY: MedianFilter = MedianFilter(10)

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer()

        //        for (i in 5800..5808) {
        //            PortForwarder.add(i, "limelight.local", i)
        //        }

        //        swo.fieldPosition = Vector3(0.0, 5.0, 0.0)

        limelightLeft.pipeline = 1
        limelightRight.pipeline = 1

        CameraServer.startAutomaticCapture()

        driverTab.add("Field", field2d)

        limelightRight.setLED(LedMode.ForceOff)
        limelightLeft.setLED(LedMode.ForceOff)

        for (autoSequence in AutoSequenceType.values()) {
            autoChooser.addOption(autoSequence.name, autoSequence)
        }

        autoChooser.setDefaultOption("None", AutoSequenceType.None)

        driverTab.add("Auto Sequence", autoChooser)
        // driverTab.add("LL", limelightBack.feed)
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    private var lastLLLightTime = 0.0
    private var lastRightLLResetTime = 0.0
    private var lastLeftLLResetTime = 0.0

    fun resetOdometryLL() {
//        if (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) {
//            return
//        }
//
//        if (!DriverStation.isTeleopEnabled()) {
//            return
//        }

        if (!odometryResetLLShuffle.getBoolean(true)) {
            return
        }

        limelightLeft.pipeline = 1
        limelightRight.pipeline = 1

        //        limelightLeft.pipeline = limelightLeft.fiducialPipeline
        //        limelightRight.pipeline = limelightRight.fiducialPipeline

        val positions: MutableList<FieldPosition> = mutableListOf()

        val backPosition = limelightLeft.getBotFieldPosition()
        if (!backPosition.isEmpty &&
                MiscCalculations.getCurrentTime() - lastLeftLLResetTime >= .05 &&
                limelightLeft.pipeline == limelightLeft.fiducialPipeline
        ) {
            val pos = backPosition.get()
            positions.add(pos)
        } else if (backPosition.isEmpty) {
            lastLeftLLResetTime = MiscCalculations.getCurrentTime()
        }

        val frontPosition = limelightRight.getBotFieldPosition()
        if (!frontPosition.isEmpty &&
                MiscCalculations.getCurrentTime() - lastRightLLResetTime >= .05 &&
                limelightRight.pipeline == limelightRight.fiducialPipeline
        ) {
            val pos = frontPosition.get()
            positions.add(pos)

        } else if (frontPosition.isEmpty) {
            lastRightLLResetTime = MiscCalculations.getCurrentTime()
        }


        if (positions.size != 0) {
            //NOTE: this is the filter for positions with the ll
            val totalPos: Vector3 = Vector3(0.0, 0.0, 0.0)
            var count = 0
            for (position in positions) {
                totalPos.x += position.x
                totalPos.y += position.y
                count++
            }
            swo.fieldPosition = Vector3(totalPos.x / count, totalPos.y / count)
        }


        SmartDashboard.putBoolean("Right has targ", limelightRight.hasTarget())
        SmartDashboard.putBoolean("Left has targ", limelightLeft.hasTarget())
        SmartDashboard.putNumber(
                "Time - Left Reset TIme",
                MiscCalculations.getCurrentTime() - lastLeftLLResetTime
        )
        SmartDashboard.putNumber(
            "Time - Right Reset TIme",
            MiscCalculations.getCurrentTime() - lastRightLLResetTime
        )

        if (DriverStation.isAutonomousEnabled() || DriverStation.isTeleopEnabled()) {
            return
        }

        if (limelightLeft.hasTarget() || limelightRight.hasTarget()) {
            // LED Code
            //                setLED(255, 0, 0, ledBuffer, led)
            if (MiscCalculations.getCurrentTime() - lastLLLightTime >= .5) {
                if (limelightLeft.hasTarget()) {
                    limelightLeft.setLED(LedMode.ForceOn)
                }
                if (limelightRight.hasTarget()) {
                    limelightRight.setLED(LedMode.ForceOn)
                }
                lastLLLightTime = MiscCalculations.getCurrentTime()
            }
        } else {
            if (MiscCalculations.getCurrentTime() - lastLLLightTime >= .5) {
                limelightRight.setLED(LedMode.ForceOff)
                limelightLeft.setLED(LedMode.ForceOff)
                lastLLLightTime = MiscCalculations.getCurrentTime()
            }
        }
    }

    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        swo.updatePosition()

//        SmartDashboard.putNumber("swo x", swo.fieldPosition.x)
//        SmartDashboard.putNumber("swo y", swo.fieldPosition.y)
//        SmartDashboard.putNumber("gyro", gyro.getYaw())

        //        pipIndex = (pipIndex + 1) % 3
        //        limelightBack.pipeline = pipIndex
        //                limelightFront.setPipeline(pipIndex)

        //        val conePos = limelightBack.getPosition(swo, 0.165, gyro)
        //        if (!conePos.isEmpty) {
        //            SmartDashboard.putNumber("Limelight Pos X", conePos.get().x)
        //            SmartDashboard.putNumber("Limelight Vert Offset",
        // limelightBack.getVerticalOffset().get())
        //            SmartDashboard.putNumber("Limelight Pos Y", conePos.get().y)
        //        }

        //        if (armSystem.desiredTilt) {
        //            limelightFront.cameraAngle = 0.0
        //            limelightBack.cameraAngle = 0.0
        //        } else {
        //            limelightFront.cameraAngle = -24.4
        //            limelightBack.cameraAngle = 24.4
        //        }

        resetOdometryLL()

        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode. */
    override fun disabledInit() {}

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class. */
    override fun autonomousInit() {
//        armSystem.initialize()
//        armSystem.autoMode = true
        swo.fieldPosition = Vector3(0.0, 0.0, 0.0)
        //        armSystem.brakeSolenoid.set(true)
        autoCommand = AutoSequence(auto, gyro, armSystem, autoPathManager, swerveDriveTrain, clawSystem, autoChooser.selected)
//        autoCommand?.schedule()

//        autoCommand = PlaceAndBalanceMid(auto, gyro, armSystem, autoPathManager, swerveDriveTrain, clawSystem)


//        autoCommand = PlaceAndBalanceMid(auto, gyro, armSystem, autoPathManager, swerveDriveTrain, clawSystem)
        autoCommand?.schedule()
        //         autoPathManager.paths["ComplexPath"]!!.schedule()

        limelightRight.setLED(LedMode.ForceOn)
        limelightLeft.setLED(LedMode.ForceOn)
    }

    /** This function is called periodically during autonomous. */
    override fun autonomousPeriodic() {
        clawSystem.run()
        armSystem.run()
    }

    /** This function is called once when teleop is enabled. */
    override fun teleopInit() {
//        armSystem.initialize()
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before
        // cancelling it
        autonomousCommand?.cancel()

        gyro.setYawOffset()
        //        gyro.setYawOffset()

        teleopArmCommand.schedule()
        teleopSwerveCommand.schedule()
        teleopClawCommand.schedule()

        limelightRight.setLED(LedMode.ForceOff)
        limelightLeft.setLED(LedMode.ForceOff)
    }

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {

        //        swerveDriveTrain.drive(Vector2(IO.moveX, IO.moveY), IO.moveTwist)
        //        SmartDashboard.putNumber("Arm Angle", armSystem.getArmAngle())

    }

    /** This function is called once when test mode is enabled. */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
//        teleopSwerveCommand.recheckDS()
//        CommandScheduler.getInstance().cancelAll()

        //        var led = AddressableLED(6)
        //        var ledBuffer = AddressableLEDBuffer(60)
        //        led.setLength(ledBuffer.length)
        //
        //        for (i in 0 until ledBuffer.length) {
        //            ledBuffer.setRGB(i, 255, 255, 255)
        //        }
        //
        //        led.setData(ledBuffer)
        //        led.start()
    }

    /** This function is called periodically during test mode. */
    override fun testPeriodic() {
        // val encoderValues =
        //     arrayOf(
        //         backLeft.getRawEncoder(),
        //         frontLeft.getRawEncoder(),
        //         frontRight.getRawEncoder(),
        //         backRight.getRawEncoder()
        //     )

        SmartDashboard.putNumber("Arm Pivot Encoder Raw", armSystem.rawArmEncoder)
        SmartDashboard.putNumber("Arm Pivot Encoder", armSystem.armAngleDegrees)
        SmartDashboard.putBoolean("Arm Extended Switch", armSystem.extensionOutBeamBreak.get())
        SmartDashboard.putBoolean("Arm Mid Switch", armSystem.extensionMidBeamBreak.get())
        SmartDashboard.putBoolean("Arm Retracted Switch", armSystem.extensionInBeamBreak.get())
//        SmartDashboard.putBoolean("Claw Break Beam", clawSystem.intakeBeamBreak.get())
        val encoderValues = arrayOf(backLeft.getRawEncoder(), frontLeft.getRawEncoder(),
                frontRight.getRawEncoder(), backRight.getRawEncoder())
        //
//        SmartDashboard.putString("Encoder Offsets", encoderValues.joinToString(", "))

        SmartDashboard.putNumber("gyro roll", gyro.getRoll())
        SmartDashboard.putNumber("gyro pitch", gyro.getPitch())


    }
}
