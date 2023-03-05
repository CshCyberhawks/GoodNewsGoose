package frc.robot.util

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

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
        get() = MiscCalculations.calculateDeadzone(rightJoy.x, .05)

    val moveY
        get() = MiscCalculations.calculateDeadzone(rightJoy.y, .05)

    val moveTwist
        get() = if (hosas) {
            MiscCalculations.calculateDeadzone(leftJoy.x, .05)
        } else MiscCalculations.calculateDeadzone(rightJoy.twist, .05)


    private var lastToggleTraversal = false
    val toggleTraversal: Boolean
        get() {
            val current = xbox.aButton
            val toggled = current && !lastToggleTraversal
            lastToggleTraversal = current
            return toggled
        }

    val toggleGrabber: Boolean
        get() = xbox.xButton

    val toggleBrake: Boolean
        get() = xbox.bButton

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
            val current = leftJoy.getRawButton(4)
            val toggled = current && !limelightAngleLockLast
            limelightAngleLockLast = current
            return toggled
        }

    private var limelightTranslateLast = false
    val limelightTranslate: Boolean
        get() {
            val current = leftJoy.getRawButton(5)
            val toggled = current && !limelightTranslateLast
            limelightTranslateLast = current
            return toggled
        }

    private var limelightSingleAxisXLast = false
    val limelightTranslateSingleAxisX: Boolean
        get() {
            val current = leftJoy.getRawButton(6)
            val toggled = current && !limelightSingleAxisXLast
            limelightSingleAxisXLast = current
            return toggled
        }

    private var limelightSingleAxisYLast = false
    val limelightTranslateSingleAxisY: Boolean
        get() {
            val current = leftJoy.getRawButton(7)
            val toggled = current && !limelightSingleAxisYLast
            limelightSingleAxisYLast = current
            return toggled
        }

    private var toggleLimelightLast = false
    val toggleLimelight: Boolean
        get() {
            val current = rightJoy.getRawButton(2)
            val toggled = current && !toggleLimelightLast
            toggleLimelightLast = current
            return toggled
        }

    val disableFieldOrientation
        get() = rightJoy.getRawButton(1)

    val pip0
        get() = leftJoy.getRawButton(5)

    val pip1
        get() = leftJoy.getRawButton(6)

    val pip2
        get() = leftJoy.getRawButton(7)

    val pip3
        get() = leftJoy.getRawButton(10)
}
