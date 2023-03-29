package cshcyberhawks.swolib.autonomous.commands

import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj2.command.CommandBase

class LockWheels(val driveTrain: SwerveDriveTrain) : CommandBase() {
    override fun initialize() {
        driveTrain.lockWheels()
    }

    override fun end(interrupted: Boolean) {
        driveTrain.kill()
    }

    override fun isFinished(): Boolean {
        return true
    }
}