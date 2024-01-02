package frc.robot.commands.auto.arm

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.Wait
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.AngleMovement
import frc.robot.commands.auto.AutoClaw
import frc.robot.commands.auto.ExtensionMovement
import frc.robot.commands.auto.TiltMovement
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

public class AutoPickupFloorCube(swerveSystem: SwerveDriveTrain, swerveAuto: SwerveAuto, armSystem: ArmSystem) : SequentialCommandGroup() {
    init {
        addCommands(
                AutoAlignFloorCube(armSystem),
                Wait(0.5),
                AutoDriveUntilCube(swerveSystem, swerveAuto, armSystem),
                AutoAlignClosed(armSystem)
        )
    }
}