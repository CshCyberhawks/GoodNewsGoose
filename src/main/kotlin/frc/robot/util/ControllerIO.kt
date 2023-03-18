package frc.robot.util

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.XboxController

object ControllerIO {
    private val xbox = XboxController(2)

    private var lastToggleGrabber = false
    val toggleGrabber: Boolean
        get() {
            val current = xbox.rightBumper
            val toggled = current && !lastToggleGrabber
            lastToggleGrabber = current
            return toggled
        }

    val toggleBrake: Boolean
        get() = xbox.leftBumper

    private var toggleTiltLast = false
    val toggleTilt: Boolean
        get() {
            val current = xbox.aButton
            val toggled = current && !toggleTiltLast
            toggleTiltLast = current
            return toggled
        }

    val controlArmAngle
        get() = MiscCalculations.calculateDeadzone(-xbox.leftY, 0.1)

    val traversalManualControl
        get() = MiscCalculations.calculateDeadzone(xbox.rightY, 0.1)

    private var toggleTraversalLast = false

    val traversalToggle: Boolean
        get() {
            val current = xbox.xButton
            val toggled = current && !toggleTraversalLast
            toggleTraversalLast = current
            return toggled
        }
}