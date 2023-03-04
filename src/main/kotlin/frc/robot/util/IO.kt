package frc.robot.util

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController

object IO {
    private val rightJoy = Joystick(0)
    private val leftJoy = Joystick(1)
    private val xbox = XboxController(2)

    private const val hosas = true

    val moveX
        get() = MiscCalculations.calculateDeadzone(rightJoy.x, 0.1)

    val moveY
        get() = MiscCalculations.calculateDeadzone(rightJoy.y, 0.1)

    val moveTwist
        get() = if (hosas) {
            leftJoy.x
        } else rightJoy.twist


    val toggleTraversal: Boolean
        get() = xbox.aButton

    val controlArmAngle
        get() = MiscCalculations.calculateDeadzone(-xbox.leftY, 0.1)
}