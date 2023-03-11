package cshcyberhawks.swolib.autonomous.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.FieldPosition
import edu.wpi.first.wpilibj2.command.CommandBase

class LimeLightAuto(val swerveAuto: SwerveAuto, val limelight: Limelight, private val targetHeight: Double) : CommandBase() {
    init {
        addRequirements(swerveAuto.swerveSystem)
    }

    override fun initialize() {
        swerveAuto.desiredPosition =
            FieldPosition(limelight.getPosition(swerveAuto.swo, targetHeight, swerveAuto.gyro), 0.0)
        swerveAuto.setDesiredAngleRelative(limelight.getHorizontalOffset())
    }

    override fun execute() {
        swerveAuto.move()
    }

    override fun isFinished(): Boolean {
        return swerveAuto.isFinishedMoving()
    }

    override fun end(int: Boolean) {
        swerveAuto.kill()
    }
}
