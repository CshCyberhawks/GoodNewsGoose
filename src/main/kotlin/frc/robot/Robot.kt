package frc.robot

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.hardware.implementations.Pigeon2Gyro
import cshcyberhawks.swolib.hardware.implementations.SparkMaxTurnMotor
import cshcyberhawks.swolib.hardware.implementations.TalonFXDriveMotor
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.Vector3
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import cshcyberhawks.swolib.swerve.SwerveOdometry
import cshcyberhawks.swolib.swerve.SwerveWheel
import cshcyberhawks.swolib.swerve.configurations.FourWheelSwerveConfiguration
import cshcyberhawks.swolib.swerve.configurations.SwerveModuleConfiguration
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.TeleopSwerveCommand
import frc.robot.commands.TestingAuto
import frc.robot.constants.MotorConstants

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {

    val driverTab = Shuffleboard.getTab("Driver")

    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null
    var pipIndex = 0
    val swerveConfiguration: SwerveModuleConfiguration = SwerveModuleConfiguration(4.0, 0.0505, 7.0)

    val drivePIDBackLeft = PIDController(0.01, 0.0, 0.0)
    val turnPIDBackLeft = PIDController(.012, 0.0, 0.0002)

    val drivePIDBackRight = PIDController(0.01, 0.0, 0.0)
    val turnPIDBackRight = PIDController(.012, 0.0, 0.0002)

    val drivePIDFrontLeft = PIDController(0.01, 0.0, 0.0)
    val turnPIDFrontLeft = PIDController(.012, 0.0, 0.0002)

    val drivePIDFrontRight = PIDController(0.01, 0.0, 0.0)
    val turnPIDFrontRight = PIDController(.012, 0.0, 0.0002)

    val limelightBack = Limelight("limelight-back", 0.12, 0.0)
    val limelightFront = Limelight("limelight-front", 0.12, 0.0)
    var backLeft: SwerveWheel =
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
    var backRight: SwerveWheel =
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
    var frontLeft: SwerveWheel =
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
    var frontRight: SwerveWheel =
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

    val gyro = Pigeon2Gyro(15)

    val swerveDriveTrain =
            SwerveDriveTrain(
                    FourWheelSwerveConfiguration(frontRight, frontLeft, backRight, backLeft),
                    gyro
            )

    val swo =
            SwerveOdometry(swerveDriveTrain, gyro, 1.0, Vector3(0.0, 0.0, 0.0), debugLogging = true)

    var autoPIDX = PIDController(.01, 0.0, 0.0)
    var autoPIDY = PIDController(.01, 0.0, 0.0)

    var auto =
            SwerveAuto(
                    autoPIDX,
                    autoPIDY,
                    PIDController(.5, 0.0, 0.05),
                    // TrapezoidProfile.Constraints(4.0, 1.5),
                    TrapezoidProfile.Constraints(1.0, .2),
                    1.0, // TODO: Tune PIDs so this can be smaller
                    0.2,
                    1.0,
                    swo,
                    swerveDriveTrain,
                    gyro,
                    true
            )

    var teleopCommand =
            TeleopSwerveCommand(
                    swerveDriveTrain,
                    auto,
                    gyro,
                    driverTab,
                    limelightFront,
                    limelightBack
            )
    var autoCommand = TestingAuto(auto, gyro)
    val autoPathManager = AutoPathManager(auto, gyro)

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        //        PortForwarder.add(5800, "limelight.local", 5800)
        for (i in 5800..5808) {
            PortForwarder.add(i, "limelight.local", i)
        }
        robotContainer = RobotContainer()
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
        CommandScheduler.getInstance().run()
    }

    /** This function is called once each time the robot enters Disabled mode. */
    override fun disabledInit() {}

    /** This function is called periodically when disabled. */
    override fun disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class. */
    override fun autonomousInit() {
        swo.fieldPosition = Vector3(0.0, 0.0, 0.0)

        autoCommand = TestingAuto(auto, gyro)
        autoCommand.schedule()
        // autoPathManager.paths["Path"]!!.schedule()
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

        teleopCommand.schedule()
    }

    /** This function is called periodically during operator control. */
    override fun teleopPeriodic() {
        pipIndex += 1
        if (pipIndex == 3) {
            pipIndex = 0
        }
        limelightBack.setPipeline(pipIndex)
        limelightFront.setPipeline(pipIndex)
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
    }
}
