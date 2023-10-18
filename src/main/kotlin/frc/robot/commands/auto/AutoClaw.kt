package frc.robot.commands.auto

import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Polar
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem


class AutoClaw(val clawSystem: ClawSystem, private val clawState: ClawState) : CommandBase() {

    override fun initialize() {
        clawSystem.clawState = clawState
    }


    override fun execute() {
    }

    override fun end(interrupted: Boolean) {

    }

    override fun isFinished(): Boolean = true
}
