package frc.robot.commands

import TeleopLimelight
import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.MiscConstants
import frc.robot.util.IO

class TeleopSwerveCommand(
        private var swerveDriveTrain: SwerveDriveTrain,
        val swerveAuto: SwerveAuto,
        var gyro: GenericGyro,
        val limelight1: Limelight,
        val limelight2: Limelight
) : CommandBase() {

    var throttle = 0.6
    var prevJoyMoveyThrottle = 0.0

    var currentLimelight = limelight1

    var currentCommand: CommandBase? = null

    init {
        addRequirements(swerveDriveTrain)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        //        Gyro.setOffset()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        var fieldOriented = true

        if (IO.killCommand) {
            currentCommand?.cancel()
            currentCommand = null
        }

        if (currentCommand != null && currentCommand?.isFinished() == false) {
            return
        }

        if (IO.gyroReset) {
            gyro.setYawOffset()
        }

        SmartDashboard.putBoolean("gyro reset button: ", IO.gyroReset)

        if (IO.fastThrottle) {
            throttle = 0.9
        }

        if (IO.normalThrottle) {
            throttle = 0.4
        }

        val quickThrottle = IO.quickThrottle
        if (quickThrottle in 135..225) {
            throttle -= MiscConstants.quickThrottleChange
        } else if (quickThrottle == 315 || quickThrottle == 45 || quickThrottle == 0) {
            throttle += MiscConstants.quickThrottleChange
        }

        if (MiscCalculations.calculateDeadzone(IO.moveyThrottle - prevJoyMoveyThrottle, .005) != 0.0
        ) {
            throttle = IO.moveyThrottle
        }

        MathUtil.clamp(throttle, 0.0, 1.0)

        var driveVec = Vector2(IO.moveX * throttle, -IO.moveY * throttle)
        var driveTwist = IO.moveTwist * throttle

        if (IO.toggleLimelight) {
            currentLimelight = if (currentLimelight == limelight1) limelight2 else limelight1
        }

        Limelight.openCamera(currentLimelight)

        if (IO.limelightAngleLock) {
            driveTwist =
                    MiscCalculations.calculateDeadzone(currentLimelight.getHorizontalOffset(), .5) /
                            32
        } else if (IO.limelightTranslate) {
            currentCommand = TeleopLimelight(currentLimelight, swerveDriveTrain)
            return
        } else if (IO.limelightTranslateSingleAxisX) {
            currentCommand =
                    AutoLimelightSingleAxis(
                            swerveAuto,
                            currentLimelight,
                            1.0,
                            AutoLimelightSingleAxis.Axis.X
                    )
            return
        } else if (IO.limelightTranslateSingleAxisY) {
            currentCommand =
                    AutoLimelightSingleAxis(
                            swerveAuto,
                            currentLimelight,
                            1.0,
                            AutoLimelightSingleAxis.Axis.Y
                    )
            return
        }

        if (IO.disableFieldOrientation) {
            fieldOriented = false
        }

        prevJoyMoveyThrottle = IO.moveyThrottle

        SmartDashboard.putNumber("throttle", throttle)

        swerveDriveTrain.drive(driveVec, driveTwist, fieldOriented)
    }
}
