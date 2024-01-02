package frc.robot.commands.auto.arm

import cshcyberhawks.swolib.autonomous.commands.Wait
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.*
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

class AutoPlaceHybrid(armSystem: ArmSystem) : SequentialCommandGroup() {
    init {
        addCommands(
//            AutoAlignHigh(armSystem),
//            AutoClaw(clawSystem, ClawState.Spitting),
//            Wait(0.5),
//            AutoClaw(clawSystem, ClawState.Idle),
//            AutoAlignClosed(armSystem)
                AutoArmPosition(armSystem, listOf(
                        AngleMovement(armSystem, ArmConstants.armInAngle + 10),
                        ExtensionMovement(armSystem, ArmConstants.armExtensionMid)
                )),
                AutoClaw(ClawState.Spitting),
        )
    }
}