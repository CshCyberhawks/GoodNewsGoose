package frc.robot

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.field2d.Field2d
import cshcyberhawks.swolib.field2d.FieldObject2d
import cshcyberhawks.swolib.hardware.implementations.Pigeon2Gyro
import cshcyberhawks.swolib.hardware.implementations.SparkMaxTurnMotor
import cshcyberhawks.swolib.hardware.implementations.TalonFXDriveMotor
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.Vector3
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import cshcyberhawks.swolib.swerve.SwerveOdometry
import cshcyberhawks.swolib.swerve.SwerveWheel
import cshcyberhawks.swolib.swerve.configurations.FourWheelAngleConfiguration
import cshcyberhawks.swolib.swerve.configurations.FourWheelSwerveConfiguration
import cshcyberhawks.swolib.swerve.configurations.SwerveModuleConfiguration
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.cscore.HttpCamera
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.net.PortForwarder
import frc.robot.commands.TeleopSwerveCommand
import frc.robot.commands.TestingAuto
import frc.robot.constants.MotorConstants
import java.util.*

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

    private val limelightBack = Limelight("limelight-back", 0.134, 0.0, fiducialPipeline = 2)

    //    private val limelightFront = Limelight("limelight-front", 0.12, 0.0, fiducialPipeline = 0)
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

    val gyro = Pigeon2Gyro(30)

    private val swerveDriveTrain =
        SwerveDriveTrain(
            FourWheelSwerveConfiguration(
                frontRight,
                frontLeft,
                backRight,
                backLeft,
                angleConfiguration =
                FourWheelAngleConfiguration(131.6, -131.6, 48.4, -48.4)
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
            arrayOf(limelightBack),
            debugLogging = true,
            field2d = Optional.of(field2d)
        )

    //    val autoTrapConstraints = TrapezoidProfile.Constraints(4.0, 1.0)
    private val autoTrapConstraints = TrapezoidProfile.Constraints(4.0, 1.0)
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
            10.0, // TODO: Tune PIDs so this can be smaller
            0.2,
            swo,
            swerveDriveTrain,
            gyro,
            true,
            Optional.of(field2d)
        )

    private var teleopCommand =
        TeleopSwerveCommand(
            swerveDriveTrain,
            auto,
            gyro,
            driverTab,
            arrayOf(limelightBack)
        )

    //    val armSystem = ArmSubsystem(driverTab)

    var autoCommand = TestingAuto(auto, gyro, limelightBack)
    private val autoPathManager = AutoPathManager(auto, gyro)

    lateinit var llCam: HttpCamera



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

        limelightBack.pipeline = 0
//        limelightFront.pipeline = 0

        driverTab.add("Field", field2d)

        // driverTab.add("LL", limelightBack.feed)
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        swo.updatePosition()
        SmartDashboard.putNumber("swo x", swo.fieldPosition.x)
        SmartDashboard.putNumber("swo y", swo.fieldPosition.y)
        SmartDashboard.putNumber("gyro", gyro.getYaw())

//        pipIndex = (pipIndex + 1) % 3
//        limelightBack.pipeline = pipIndex
//                limelightFront.setPipeline(pipIndex)

        val conePos = limelightBack.getPosition(swo, 0.165, gyro)
        if (!conePos.isEmpty) {
            SmartDashboard.putNumber("Limelight Pos X", conePos.get().x)
            SmartDashboard.putNumber("Limelight Vert Offset", limelightBack.getVerticalOffset().get())
            SmartDashboard.putNumber("Limelight Pos Y", conePos.get().y)
        }

        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode. */
    override fun disabledInit() {}

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class. */
    override fun autonomousInit() {
        swo.fieldPosition = Vector3(0.0, 0.0, 0.0)
        //        armSystem.brakeSolenoid.set(true)

        autoCommand = TestingAuto(auto, gyro, limelightBack)
        autoCommand.schedule()
        // autoPathManager.paths["Path"]!!.schedule()
        // autoPathManager.paths["ComplexPath"]!!.schedule()
    }

    /** This function is called periodically during autonomous. */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before
        // cancelling it
        autonomousCommand?.cancel()
        gyro.setYawOffset()

        //        val armCommand = ManualArmCommand(armSystem)
        //        armCommand.schedule()
        teleopCommand.schedule()
    }

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {

        //        swerveDriveTrain.drive(Vector2(IO.moveX, IO.moveY), IO.moveTwist)
        //        SmartDashboard.putNumber("Arm Angle", armSystem.getArmAngle())
    }

    /** This function is called once when test mode is enabled. */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode. */
    override fun testPeriodic() {
        val encoderValues =
            arrayOf(
                backLeft.getRawEncoder(),
                frontLeft.getRawEncoder(),
                frontRight.getRawEncoder(),
                backRight.getRawEncoder()
            )

        SmartDashboard.putString("Encoder Offsets", encoderValues.joinToString(", "))
        //        val encoderValues = arrayOf(backLeft.getRawEncoder(), frontLeft.getRawEncoder(),
        // frontRight.getRawEncoder(), backRight.getRawEncoder())
        //
        //        SmartDashboard.putString("Encoder Offsets", encoderValues.joinToString(", "))
    }
}
