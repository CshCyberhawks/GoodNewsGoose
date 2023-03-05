package frc.robot.util

import edu.wpi.first.wpilibj.Joystick
import cshcyberhawks.swolib.math.MiscCalculations

object IO {
    private val rightJoy = Joystick(0)
    private val leftJoy = Joystick(1)

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

    val killCommand
        get() = leftJoy.getRawButtonPressed(2)

    val limelightAngleLock
        get() = rightJoy.getRawButtonPressed(4)
    
    val limelightTranslate
        get() = rightJoy.getRawButtonPressed(5)

    val limelightTranslateSingleAxisX
        get() = rightJoy.getRawButtonPressed(6)

    val limelightTranslateSingleAxisY
        get() = rightJoy.getRawButtonPressed(7)

    val toggleLimelight
        get() = rightJoy.getRawButtonPressed(2)

    val disableFieldOrientation
        get() = rightJoy.getRawButtonPressed(1)
}
