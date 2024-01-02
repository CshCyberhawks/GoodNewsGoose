package frc.robot.commands.auto.arm

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem


class AutoDriveUntilCube(val swerveSystem: SwerveDriveTrain, val swerveAuto: SwerveAuto, val armSystem: ArmSystem) : CommandBase() {
    var startTime = -1.0

    override fun initialize() {
        ClawSystem.clawState = ClawState.Intaking
        startTime = MiscCalculations.getCurrentTimeSeconds()
    }


    override fun execute() {
        if (MiscCalculations.getCurrentTimeSeconds() - startTime < 1.82) {
            swerveSystem.drive(Vector2(0.0, 0.2), 0.0, true)
        } else {
            swerveSystem.drive(Vector2(0.0, 0.0), 0.0, true)
        }
    }

    override fun end(interrupted: Boolean) {
        swerveSystem.drive(Vector2(0.0, 0.0), 0.0, true)
    }

    override fun isFinished(): Boolean {
        return ClawSystem.clawState == ClawState.Idle
    }
}
