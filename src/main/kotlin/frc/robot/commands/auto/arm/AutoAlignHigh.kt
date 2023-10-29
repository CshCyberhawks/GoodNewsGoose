package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.AngleMovement
import frc.robot.commands.auto.ExtensionMovement
import frc.robot.commands.auto.TiltMovement
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem

public class AutoAlignHigh(private val armSystem: ArmSystem): CommandBase() {

    var cmd: AutoArmPosition? = null;
    init {
//        SmartDashboard.putBoolean("inited test cmd",false)

    }

    override fun initialize() {
//        SmartDashboard.putBoolean("inited test cmd", true)
        this.cmd = AutoArmPosition(armSystem, listOf(
            AngleMovement(armSystem, ArmConstants.armMidAngle),
            TiltMovement(armSystem, true),
            AngleMovement(armSystem, ArmConstants.armHighAngle),
            ExtensionMovement(armSystem, ArmConstants.armExtensionOut)
        ))
        cmd!!.schedule()
    }

    override fun isFinished(): Boolean {
//        SmartDashboard.putBoolean("isFinished", cmd!!.isFinished)
        return cmd!!.isFinished
    }
}