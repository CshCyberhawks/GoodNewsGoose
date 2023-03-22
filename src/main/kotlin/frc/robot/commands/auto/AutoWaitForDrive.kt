package frc.robot.commands.auto

import cshcyberhawks.swolib.autonomous.SwerveAuto
import edu.wpi.first.wpilibj2.command.CommandBase

class AutoWaitForDrive(private val swerveAuto: SwerveAuto) : CommandBase() {
    override fun isFinished(): Boolean {
        return swerveAuto.isFinishedMoving()
    }
}