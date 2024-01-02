package frc.robot.commands.auto.arm

import cshcyberhawks.swolib.autonomous.commands.Wait
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.*
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

class AutoPlaceHigh(armSystem: ArmSystem) : SequentialCommandGroup() {
    init {
        addCommands(
//            AutoAlignHigh(armSystem),
//            AutoClaw(clawSystem, ClawState.Spitting),
//            Wait(0.5),
//            AutoClaw(clawSystem, ClawState.Idle),
//            AutoAlignClosed(armSystem)
                AutoArmPosition(armSystem, listOf(
                        AngleMovement(armSystem, ArmConstants.armMidAngle),
                        TiltMovement(armSystem, true),
                        AngleMovement(armSystem, ArmConstants.armHighAngle),
                        ExtensionMovement(armSystem, ArmConstants.armExtensionOut),
                        AngleMovement(armSystem, ArmConstants.armPlaceHighAngle),
                )),
                AutoClaw(ClawState.Spitting),
                AutoArmPosition(armSystem, listOf(
                        AngleMovement(armSystem, ArmConstants.armPlaceHighAngle - ArmConstants.armPlaceAngleDecrease),
                        ExtensionMovement(armSystem, ArmConstants.armExtensionIn),
                        TiltMovement(armSystem, false),
                        AngleMovement(armSystem, ArmConstants.armInAngle)
                )),
                AutoClaw(ClawState.Idle)
        )
    }
}