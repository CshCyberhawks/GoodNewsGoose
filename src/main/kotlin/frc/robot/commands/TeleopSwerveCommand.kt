package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.math.Vector3
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.math.MathUtil
import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.DriverPreferences
import frc.robot.util.JoyIO

class TeleopSwerveCommand(
        private var swerveDriveTrain: SwerveDriveTrain,
        val swerveAuto: SwerveAuto,
        var gyro: GenericGyro,
        driverTab: ShuffleboardTab,
        private val limelightArray: Array<Limelight>
) : CommandBase() {

    private var desiredPipe = 0

    private var throttle = 0.6
    private var prevJoyMoveyThrottle = 0.0

    private var currentLimelight = limelightArray[0]
    private var currentLimelightIndex = 0

    private var currentCommand: CommandBase? = null

    private val throttleShuffle: GenericEntry = driverTab.add("Throttle", 0.0).entry
    private val pipShuffle: GenericEntry = driverTab.add("Desired PIP", 0).entry
    private val currentLimelightShuffle: GenericEntry =
            driverTab.add("Current Limelight Name", "Unknown").entry

    private var presetPositions: Array<FieldPosition>

    private val yForAutoThing = 1.93

    init {
        addRequirements(swerveDriveTrain)
        // 1 was originally 3.83 +.24
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            SmartDashboard.putString("ds alliance", "blue")

            this.presetPositions =
                    arrayOf(
                            FieldPosition(-4.82, yForAutoThing, 180.0),
                            FieldPosition(-3.83, yForAutoThing, 180.0),
                            FieldPosition(-3.28, yForAutoThing, 180.0),
                            FieldPosition(-2.20, yForAutoThing, 180.0),
                            FieldPosition(-1.62, yForAutoThing, 180.0),
                            FieldPosition(-.52, yForAutoThing, 180.0)
                    )
        } else {
            SmartDashboard.putString("ds alliance", "red")

            this.presetPositions =
                    arrayOf(
                            FieldPosition(4.82, yForAutoThing, 180.0),
                            FieldPosition(3.83, yForAutoThing, 180.0),
                            FieldPosition(3.28, yForAutoThing, 180.0),
                            FieldPosition(2.20, yForAutoThing, 180.0),
                            FieldPosition(1.62, yForAutoThing, 180.0),
                            FieldPosition(.52, yForAutoThing, 180.0)
                    )
        }
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        //        Gyro.setOffset()
    }

    private fun setCurrentCommand(command: CommandBase) {
        this.currentCommand = command
        this.currentCommand?.schedule()
    }

    private fun setCurrentCommand(command1: CommandBase, command2: CommandBase) {
        this.currentCommand = command1
        this.currentCommand?.andThen(command2)?.schedule()
    }

    fun recheckDS() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            SmartDashboard.putString("ds alliance", "blue")

            this.presetPositions =
                    arrayOf(
                            FieldPosition(-4.82, yForAutoThing, 180.0),
                            FieldPosition(-3.83, yForAutoThing, 180.0),
                            FieldPosition(-3.28, yForAutoThing, 180.0),
                            FieldPosition(-2.20, yForAutoThing, 180.0),
                            FieldPosition(-1.62, yForAutoThing, 180.0),
                            FieldPosition(-.52, yForAutoThing, 180.0)
                    )
        } else {
            SmartDashboard.putString("ds alliance", "red")

            this.presetPositions =
                    arrayOf(
                            FieldPosition(4.82, yForAutoThing, 180.0),
                            FieldPosition(3.83, yForAutoThing, 180.0),
                            FieldPosition(3.28, yForAutoThing, 180.0),
                            FieldPosition(2.20, yForAutoThing, 180.0),
                            FieldPosition(1.62, yForAutoThing, 180.0),
                            FieldPosition(.52, yForAutoThing, 180.0)
                    )
        }
    }

    private fun closestPresetPos(): Vector2 {
        return MiscCalculations.closestPoint(
                Vector2(swerveAuto.swo.fieldPosition.x, swerveAuto.swo.fieldPosition.y),
                presetPositions.map { Vector2(it.x, it.y) }.toTypedArray()
        )
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (!DriverStation.isTeleopEnabled()) {
            return
        }
        throttleShuffle.setDouble(throttle)
        pipShuffle.setInteger(desiredPipe.toLong())
        currentLimelightShuffle.setString(currentLimelight.name)

        if (JoyIO.killCommand) {
            currentCommand?.cancel()
            currentCommand = null
            swerveAuto.kill()
        }

        if (currentCommand != null && currentCommand?.isFinished() == false) {
            return
        } else if (currentCommand != null && currentCommand?.isFinished() == true) {
            currentCommand?.cancel()
            currentCommand = null
        }

        //        } else if (IO.pip2) {
        //            desiredPip = 2
        //        } else if (IO.pip3) {
        //            desiredPip = 3
        //        }
        if (JoyIO.resetSwo) {
            swerveAuto.swo.fieldPosition = Vector3()
        }

        //        if (JoyIO.toggleLimelight) {
        //            currentLimelightIndex = (currentLimelightIndex + 1) % limelightArray.size
        //            currentLimelight = limelightArray[currentLimelightIndex]
        //            currentLimelight.pipeline = desiredPipe
        //        }

        //         if(JoyIO.limelightGyroCorrect) {
        //             /**assumes limelight is at center of robot*/
        //             val offset = currentLimelight.getHorizontalOffset()
        //            /**"better"*/
        // //            val offset = currentLimelight.getBotYawClose()
        //             if (offset.isPresent) {
        //                 gyro.setYawOffset(-offset.get())
        //             }
        //         }
        //                 if (JoyIO.limelightChangeRot) {
        //                     val offset = currentLimelight.getHorizontalOffset()
        //                     if (offset.isPresent) {
        //                         gyro.setYawOffset(-offset.get() + 90.0)
        //                     }
        //                 }
        if (JoyIO.gyroReset) {
            gyro.setYawOffset()
        }

        // if (JoyIO.togglePipe) {
        //     desiredPipe = if (desiredPipe == 0) 1 else 0
        //     currentLimelight.pipeline = desiredPipe
        // }

        val quickThrottle = JoyIO.quickThrottle
        if (quickThrottle in 135..225) {
            throttle -= DriverPreferences.quickThrottleChange
        } else if (quickThrottle == 45 || quickThrottle == 0 || quickThrottle == 315) {
            throttle += DriverPreferences.quickThrottleChange
        }

        if (MiscCalculations.calculateDeadzone(JoyIO.moveYThrottle - prevJoyMoveyThrottle, .005) !=
                0.0
        ) {
            throttle = JoyIO.moveYThrottle
        }

        throttle = MathUtil.clamp(throttle, 0.0, 1.0)

        val driveVec = Vector2(JoyIO.moveX * throttle, -JoyIO.moveY * throttle)
        var driveTwist = JoyIO.moveTwist * throttle

        // if (JoyIO.toggleLimelight) {
        //     currentLimelightIndex = (currentLimelightIndex + 1) % limelightArray.size
        //     currentLimelight = limelightArray[currentLimelightIndex]
        //     //            currentLimelight.pipeline = desiredPipe
        //     desiredPipe = currentLimelight.pipeline
        // }

        if (JoyIO.limelightAngleLock) {
            driveTwist =
                    MiscCalculations.calculateDeadzone(
                            currentLimelight.getHorizontalOffset().get(),
                            .5
                    ) / 50
        }

        //        if (JoyIO.limelightTranslateSingleAxisX) {
        //            val limelightOffset = currentLimelight.getHorizontalOffset()
        //            if (!limelightOffset.isEmpty) {
        //                swerveDriveTrain.drive(Vector2(limelightOffset.get() / 100, 0.0), 0.0,
        // true)
        //                return
        //            }
        //        }
        if (JoyIO.limelightTranslate) {
            setCurrentCommand(TeleopLimelight(currentLimelight, swerveDriveTrain, desiredPipe))
            return
        }
        //        if (JoyIO.limelightTranslateSingleAxisX) {
        //            SmartDashboard.putBoolean("single axis x set", true)
        //            setCurrentCommand(
        //                    AutoLimelightSingleAxis(
        //                            swerveAuto,
        //                            currentLimelight,
        //                            0.61,
        //                            AutoLimelightSingleAxis.Axis.X,
        //                            desiredPipe
        //                    )
        //            )
        //            return
        //        } else {
        //            SmartDashboard.putBoolean("single axis x set", false)
        //        }
        //        if (JoyIO.limelightTranslateSingleAxisY) {
        //            setCurrentCommand(
        //                    AutoLimelightSingleAxis(
        //                            swerveAuto,
        //                            currentLimelight,
        //                            0.61,
        //                            AutoLimelightSingleAxis.Axis.Y,
        //                            desiredPipe
        //                    )
        //            )
        //            return
        //        }

//        if (JoyIO.nearestConePos) {
//            setCurrentCommand(
//                    GoToPosition(
//                            swerveAuto,
//                            FieldPosition(
//                                    swerveAuto.swo.fieldPosition.x,
//                                    swerveAuto.swo.fieldPosition.y,
//                                    180.0
//                            )
//                    ),
//                    GoToPosition(swerveAuto, closestPresetPos())
//            )
//            return
//        }

        // SmartDashboard.putNumber("preset pos", JoyIO.presetPos.toDouble())
        if (JoyIO.presetPos != -1) {
            setCurrentCommand(
                    GoToPosition(
                            swerveAuto,
                            FieldPosition(
                                    swerveAuto.swo.fieldPosition.x,
                                    swerveAuto.swo.fieldPosition.y,
                                    180.0
                            )
                    ),
                    GoToPosition(swerveAuto, presetPositions[JoyIO.presetPos])
            )
            return
        }

        prevJoyMoveyThrottle = JoyIO.moveYThrottle

        swerveDriveTrain.drive(driveVec, driveTwist, JoyIO.disableFieldOrientation)
    }
}
