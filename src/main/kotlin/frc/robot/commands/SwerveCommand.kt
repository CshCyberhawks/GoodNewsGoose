package frc.robot.commands

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.MiscConstants
import frc.robot.util.IO

class SwerveCommand(private var swerveDriveTrain: SwerveDriveTrain, var gyro: GenericGyro) :
        CommandBase() {

    var throttle = 0.6
    var prevJoyMoveyThrottle = 0.0

    init {
        addRequirements(swerveDriveTrain)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        //        Gyro.setOffset()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
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


        // val angle = if (IO.limelightLockOn()) MathClass.calculateDeadzone(
        //         -Robot.limelight.getHorizontalOffset(),
        //         .5
        // ) / 32 else IO.turnControl()

        if (MiscCalculations.calculateDeadzone(IO.moveyThrottle - prevJoyMoveyThrottle, .005) != 0.0
        ) {
            throttle = IO.moveyThrottle
        }

        MathUtil.clamp(throttle, 0.0, 1.0)

        val driveVec = Vector2(IO.moveX * throttle, -IO.moveY * throttle)
        swerveDriveTrain.drive(
                driveVec,
                IO.moveTwist * throttle,
        )

        prevJoyMoveyThrottle = IO.moveyThrottle

        SmartDashboard.putNumber("throttle", throttle)
    }
}
