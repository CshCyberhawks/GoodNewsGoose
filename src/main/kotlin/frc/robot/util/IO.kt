package frc.robot.util

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController

object IO {
    private val rightJoy = Joystick(0)
    private val leftJoy = Joystick(1)
    private val xbox = XboxController(2)

    private const val hosas = true

    val quickThrottle
        get() = leftJoy.pov

    val normalThrottle
        get() = leftJoy.getRawButtonPressed(3)

    val fastThrottle
        get() = leftJoy.getRawButtonPressed(4)

    val moveyThrottle
        get() = if (hosas) {
            MiscCalculations.calculateDeadzone((-leftJoy.throttle + 1) / 2, .05)
        } else MiscCalculations.calculateDeadzone((-rightJoy.throttle + 1) / 2, .05)

    val gyroReset
        get() = rightJoy.getRawButtonPressed(2)

    val moveX
        get() = MiscCalculations.calculateDeadzone(rightJoy.x, .08)

    val moveY
        get() = MiscCalculations.calculateDeadzone(rightJoy.y, .08)

    val moveTwist
        get() = if (hosas) {
            MiscCalculations.calculateDeadzone(leftJoy.x, .08)
        } else MiscCalculations.calculateDeadzone(rightJoy.twist, .08)

    val resetFieldLimelight
        get() = leftJoy.getRawButtonPressed(1)

    private var limelightGyroCorrectToggled = false
    val limelightGyroCorrect: Boolean
        get() {
            limelightGyroCorrectToggled = xbox.bButtonPressed
            println(limelightGyroCorrectToggled)
            return limelightGyroCorrectToggled
        }

    val limelightChangeRot: Boolean
        get() {
            return xbox.aButtonPressed
        }
    private var lastToggleTraversal = false
    val toggleTraversal: Boolean
        get() {
            val current = xbox.aButton
            val toggled = current && !lastToggleTraversal
            lastToggleTraversal = current
            return toggled
        }

    val toggleGrabber: Boolean
        get() = xbox.rightBumper

    val toggleBrake: Boolean
        get() = xbox.leftBumper

    val controlArmAngle
        get() = MiscCalculations.calculateDeadzone(-xbox.leftY, 0.1)

    val travManualControl
        get() = xbox.rightY

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
            val current = rightJoy.getRawButton(5)
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
            val current = leftJoy.getRawButton(2)
            val toggled = current && !toggleLimelightLast
            toggleLimelightLast = current
            return toggled
        }

    val resetSwo
        get() = rightJoy.getRawButton(11)

    val disableFieldOrientation
        get() = rightJoy.getRawButton(1)

    val pipe0
        get() = leftJoy.getRawButton(5)

    val pipe1
        get() = leftJoy.getRawButton(6)

    val pipe2
        get() = leftJoy.getRawButton(7)

    val pipe3
        get() = leftJoy.getRawButton(10)
}
