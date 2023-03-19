package frc.robot.util

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.Joystick
import frc.robot.constants.DriverPreferences

object JoyIO {
    private val rightJoy = Joystick(0)
    private val leftJoy = Joystick(1)

    val quickThrottle
        get() = leftJoy.pov

//    val normalThrottle
//        get() = leftJoy.getRawButtonPressed(3)
//
//    val fastThrottle
//        get() = leftJoy.getRawButtonPressed(4)

    val moveYThrottle
        get() = if (DriverPreferences.hosas) {
            MiscCalculations.calculateDeadzone((-leftJoy.throttle + 1) / 2, .05)
        } else MiscCalculations.calculateDeadzone((-rightJoy.throttle + 1) / 2, .05)

    val gyroReset
        get() = rightJoy.getRawButtonPressed(2)

    val moveX
        get() = MiscCalculations.calculateDeadzone(rightJoy.x, .08)

    val moveY
        get() = MiscCalculations.calculateDeadzone(rightJoy.y, .08)

    val moveTwist
        get() = if (DriverPreferences.hosas) {
            MiscCalculations.calculateDeadzone(leftJoy.x, .08)
        } else MiscCalculations.calculateDeadzone(rightJoy.twist, .08)

    val resetFieldLimelight
        get() = leftJoy.getRawButtonPressed(1)


    //    val limelightGyroCorrect: Boolean
//        get() {
//            return xbox.bButtonPressed
//        }
    val limelightChangeRot: Int
        get() {
            return rightJoy.pov
        }

    private var killCommandLast = false
    val killCommand: Boolean
        get() {
            val current = leftJoy.getRawButton(2)
            val toggled = current && !killCommandLast
            killCommandLast = current
            return toggled
        }

    private var limelightAngleLockLast = false

    val limelightAngleLock: Boolean
        get() {
//            val current = leftJoy.getRawButton(4)
//            val toggled = current && !limelightAngleLockLast
//            limelightAngleLockLast = current
//            return toggled
            return rightJoy.getRawButton(4)
        }

    private var limelightTranslateLast = false
    val limelightTranslate: Boolean
        get() {
            val current = rightJoy.getRawButton(3)
            val toggled = current && !limelightTranslateLast
            limelightTranslateLast = current
            return toggled
        }

    val limelightTranslateSingleAxisX: Boolean
        get() = rightJoy.getRawButton(6)

    private var limelightSingleAxisYLast = false
    val limelightTranslateSingleAxisY: Boolean
        get() {
            val current = rightJoy.getRawButton(7)
            val toggled = current && !limelightSingleAxisYLast
            limelightSingleAxisYLast = current
            return toggled
        }

    private var toggleLimelightLast = false
    val toggleLimelight: Boolean
        get() {
            val current = leftJoy.getRawButton(3)
            val toggled = current && !toggleLimelightLast
            toggleLimelightLast = current
            return toggled
        }

    val resetSwo
        get() = rightJoy.getRawButton(11)

    val disableFieldOrientation
        get() = rightJoy.getRawButton(1)

    private var togglePipeLast = false
    val togglePipe: Boolean
        get() {
            val current = leftJoy.getRawButton(4)
            val toggled = current && !togglePipeLast
            togglePipeLast = current
            return toggled
        }
}
