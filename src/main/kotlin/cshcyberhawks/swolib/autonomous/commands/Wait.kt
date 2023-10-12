package cshcyberhawks.swolib.autonomous.commands

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase

class Wait(val waitAmountSeconds: Double) : CommandBase() {
    private var startTime = 0.0
    override fun initialize() {
        startTime = MiscCalculations.getCurrentTime()
    }

    override fun end(interrupted: Boolean) {
    }

    override fun isFinished(): Boolean {
        SmartDashboard.putNumber("wait time", MiscCalculations.getCurrentTime() - startTime)
        return (MiscCalculations.getCurrentTime() - startTime) > waitAmountSeconds
    }
}