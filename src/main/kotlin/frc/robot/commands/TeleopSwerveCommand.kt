package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.MiscConstants
import frc.robot.util.IO

class TeleopSwerveCommand(
    private var swerveDriveTrain: SwerveDriveTrain,
    val swerveAuto: SwerveAuto,
    var gyro: GenericGyro,
    driverTab: ShuffleboardTab,
    private val limelightArray: Array<Limelight>
) : CommandBase() {

    private var desiredPip = 2

    private var throttle = 0.6
    private var prevJoyMoveyThrottle = 0.0

    private var currentLimelight = limelightArray[0]
    private var currentLimelightIndex = 0

    private var currentCommand: CommandBase? = null

    private val throttleShuffle: GenericEntry = driverTab.add("Throttle", 0.0).entry
    private val pipShuffle: GenericEntry = driverTab.add("Desired PIP", 0).entry
    private val currentLimelightShuffle: GenericEntry = driverTab.add("Current Limelight IDX", 0).entry


    init {
        addRequirements(swerveDriveTrain)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        //        Gyro.setOffset()
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        throttleShuffle.setDouble(throttle)
        pipShuffle.setInteger(desiredPip.toLong())
        currentLimelightShuffle.setInteger(currentLimelightIndex.toLong())


        if (IO.killCommand) {
            currentCommand?.cancel()
            currentCommand = null
        }

        if (currentCommand != null && currentCommand?.isFinished == false) {
            if (currentCommand?.isScheduled != true) {
                currentCommand?.schedule()
            }
            return
        }

//        if (IO.pip0) {
//            desiredPip = 0
//        } else if (IO.pip1) {
//            desiredPip = 1
//        } else if (IO.pip2) {
//            desiredPip = 2
//        } else if (IO.pip3) {
//            desiredPip = 3
//        }

        if (IO.gyroReset) {
            gyro.setYawOffset()
        }

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

        val driveVec = Vector2(IO.moveX * throttle, -IO.moveY * throttle)
        var driveTwist = IO.moveTwist * throttle

        if (IO.toggleLimelight) {
            currentLimelightIndex = (currentLimelightIndex + 1) % limelightArray.size
            currentLimelight = limelightArray[currentLimelightIndex]
        }

//        Limelight.openCamera(currentLimelight)

        if (IO.limelightAngleLock) {
            driveTwist =
                MiscCalculations.calculateDeadzone(currentLimelight.getHorizontalOffset(), .5) /
                    50
        } else if (IO.limelightTranslate) {
            currentCommand = TeleopLimelight(currentLimelight, swerveDriveTrain, desiredPip)
            return
        } else if (IO.limelightTranslateSingleAxisX) {
            currentCommand =
                AutoLimelightSingleAxis(
                    swerveAuto,
                    currentLimelight,
                    1.0,
                    AutoLimelightSingleAxis.Axis.X,
                    desiredPip
                )
            return
        } else if (IO.limelightTranslateSingleAxisY) {
            currentCommand =
                AutoLimelightSingleAxis(
                    swerveAuto,
                    currentLimelight,
                    1.0,
                    AutoLimelightSingleAxis.Axis.Y,
                    desiredPip
                )
            return
        }

        prevJoyMoveyThrottle = IO.moveyThrottle

        swerveDriveTrain.drive(driveVec, driveTwist, IO.disableFieldOrientation)
    }
}
