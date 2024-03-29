package frc.robot.commands.auto.arm

import cshcyberhawks.swolib.autonomous.commands.Wait
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.AngleMovement
import frc.robot.commands.auto.AutoClaw
import frc.robot.commands.auto.ExtensionMovement
import frc.robot.commands.auto.TiltMovement
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

class AutoPlaceMid(armSystem: ArmSystem, clawSystem: ClawSystem) : SequentialCommandGroup() {
    init {
        addCommands(
//            AutoAlignMid(armSystem),
//            AutoClaw(clawSystem, ClawState.Spitting),
//            Wait(0.5),
//            AutoClaw(clawSystem, ClawState.Idle),
//            AutoAlignClosed(armSystem)
            AutoArmPosition(armSystem, listOf(
                AngleMovement(armSystem, ArmConstants.armMidAngle),
                ExtensionMovement(armSystem, ArmConstants.armExtensionMid),
                AngleMovement(armSystem, ArmConstants.armPlaceMidAngle)
            )),
            AutoClaw(clawSystem, ClawState.Spitting),
            AutoArmPosition(armSystem, listOf(
                AngleMovement(armSystem, ArmConstants.armPlaceMidAngle - ArmConstants.armPlaceAngleDecrease),
                ExtensionMovement(armSystem, ArmConstants.armExtensionIn),
                AngleMovement(armSystem, ArmConstants.armInAngle)
            )),
            AutoClaw(clawSystem, ClawState.Idle)
        )
    }
}