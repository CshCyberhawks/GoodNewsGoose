package frc.robot

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.hardware.implementations.NavXGyro
import cshcyberhawks.swolib.hardware.implementations.SparkMaxTurnMotor
import cshcyberhawks.swolib.hardware.implementations.TalonFXDriveMotor
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import cshcyberhawks.swolib.swerve.SwerveOdometry
import cshcyberhawks.swolib.swerve.SwerveWheel
import cshcyberhawks.swolib.swerve.configurations.FourWheelSwerveConfiguration
import cshcyberhawks.swolib.swerve.configurations.SwerveModuleConfiguration
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.ManualArmCommand
import frc.robot.constants.MotorConstants
import frc.robot.subsystems.ArmSubsystem
import frc.robot.util.IO

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null

    val swerveConfiguration: SwerveModuleConfiguration = SwerveModuleConfiguration(4.0, 0.0505, 7.0)

    val drivePID = PIDController(0.01, 0.0, 0.0)
    val turnPID = PIDController(0.001, 0.0, 0.0)

    var backLeft: SwerveWheel =
        SwerveWheel(
            TalonFXDriveMotor(MotorConstants.backLeftDriveMotor),
            SparkMaxTurnMotor(MotorConstants.backLeftTurnMotor, MotorConstants.backLeftEncoder, MotorConstants.turnEncoderOffsets[MotorConstants.backLeftEncoder - 10]),
            drivePID,
            turnPID,
            swerveConfiguration
        )
    var backRight: SwerveWheel =
        SwerveWheel(
            TalonFXDriveMotor(MotorConstants.backRightDriveMotor),
            SparkMaxTurnMotor(MotorConstants.backRightTurnMotor, MotorConstants.backRightEncoder, MotorConstants.turnEncoderOffsets[MotorConstants.backRightEncoder - 10]),
            drivePID,
            turnPID,
            swerveConfiguration
        )
    var frontLeft: SwerveWheel =
        SwerveWheel(
            TalonFXDriveMotor(MotorConstants.frontLeftDriveMotor),
            SparkMaxTurnMotor(MotorConstants.frontLeftTurnMotor, MotorConstants.frontLeftEncoder, MotorConstants.turnEncoderOffsets[MotorConstants.frontLeftEncoder - 10]),
            drivePID,
            turnPID,
            swerveConfiguration
        )
    var frontRight: SwerveWheel =
        SwerveWheel(
            TalonFXDriveMotor(MotorConstants.frontRightDriveMotor),
            SparkMaxTurnMotor(MotorConstants.frontRightTurnMotor, MotorConstants.frontRightEncoder, MotorConstants.turnEncoderOffsets[MotorConstants.frontRightEncoder - 10]),
            drivePID,
            turnPID,
            swerveConfiguration
        )

//    val gyro = NavXGyro(SPI.Port.kMXP)

//    val swerveDriveTrain =
//        SwerveDriveTrain(FourWheelSwerveConfiguration(frontRight, frontLeft, backRight, backLeft), gyro)
//
//    val swo = SwerveOdometry(swerveDriveTrain, gyro, 1.0)
//
//    val autoPIDX = PIDController(1.0, 0.0, 0.05)
//    val autoPIDY = PIDController(1.0, 0.0, 0.05)
//    val auto = SwerveAuto(
//        autoPIDX,
//        autoPIDY,
//        PIDController(1.5, 0.0, 0.05),
//        // TrapezoidProfile.Constraints(4.0, 1.5),
//        TrapezoidProfile.Constraints(1.0, .2),
//        10.0, // TODO: Tune PIDs so this can be smaller
//        0.2,
//        .05,
//        swo,
//        swerveDriveTrain,
//        gyro,
//    )

    val armSystem = ArmSubsystem()


    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer()
        CameraServer.startAutomaticCapture()
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    override fun disabledInit() {}

    /**
     * This function is called periodically when disabled.
     */
    override fun disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your [RobotContainer] class.
     */
    override fun autonomousInit() {
        autonomousCommand = robotContainer?.autonomousCommand

        // Schedule the autonomous command (example)
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before scheduling it
        autonomousCommand?.schedule()
    }

    /**
     * This function is called periodically during autonomous.
     */
    override fun autonomousPeriodic() {}

    /**
     * This function is called once when teleop is enabled.
     */
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before cancelling it
        autonomousCommand?.cancel()
        val armCommand = ManualArmCommand(armSystem)
        armCommand.schedule()
    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {
//        swerveDriveTrain.drive(Vector2(IO.moveX, IO.moveY), IO.moveTwist)
        SmartDashboard.putNumber("Arm Angle", armSystem.getArmAngle())
    }

    /**
     * This function is called once when test mode is enabled.
     */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /**
     * This function is called periodically during test mode.
     */
    override fun testPeriodic() {
        val encoderValues = arrayOf(backLeft.getRawEncoder(), frontLeft.getRawEncoder(), frontRight.getRawEncoder(), backRight.getRawEncoder())

        SmartDashboard.putString("Encoder Offsets", encoderValues.joinToString(", "))
    }
}
