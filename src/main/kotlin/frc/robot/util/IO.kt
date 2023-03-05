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
        get() = if (hosas) {MiscCalculations.calculateDeadzone((-leftJoy.throttle + 1) / 2, .05)} else MiscCalculations.calculateDeadzone((-rightJoy.throttle + 1) / 2, .05)

    val gyroReset
        get() = rightJoy.getRawButtonPressed(2) 

    val moveX
        get() = MiscCalculations.calculateDeadzone(rightJoy.x, .05)

    val moveY
        get() = MiscCalculations.calculateDeadzone(rightJoy.y, .05)

    val moveTwist
        get() = if (hosas) {MiscCalculations.calculateDeadzone(leftJoy.x, .05)} else MiscCalculations.calculateDeadzone(rightJoy.twist, .05)

    val toggleTraversal: Boolean
        get() = xbox.aButton

    val controlArmAngle
        get() = MiscCalculations.calculateDeadzone(-xbox.leftY, 0.1)

    val travManualControl
        get() = xbox.rightY
}
