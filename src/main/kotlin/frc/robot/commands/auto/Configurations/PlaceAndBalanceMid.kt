package frc.robot.commands.auto.Configurations

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.autonomous.commands.Wait
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.FieldPosition
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import frc.robot.commands.auto.DumbAutoBalance
import frc.robot.commands.auto.arm.AutoPlaceHigh
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawSystem

// define an empty SequentialCommandGroup
class PlaceAndBalanceMid(private val swerveAuto: SwerveAuto, private val gyro: GenericGyro, private val armSystem: ArmSystem, private val autoPathManager: AutoPathManager, private val swerveSystem: SwerveDriveTrain, private val clawSystem: ClawSystem) :
        SequentialCommandGroup() {

    // define the constructor
    init {
        gyro.setYawOffset()
        // add the commands to the SequentialCommandGroup
        addCommands(
                AutoPlaceHigh(armSystem, clawSystem),
                autoPathManager.paths["MidTaxiBalance1"]!!,
//                GoToPosition(swerveAuto, Vector2())
                Wait(4.0),
                autoPathManager.paths["MidTaxiBalance2"]!!,
                DumbAutoBalance(gyro, swerveSystem),
//            LockWheels(swerveSystem)
                GoToPosition(swerveAuto, FieldPosition(swerveAuto.swo.fieldPosition.x, swerveAuto.swo.fieldPosition.y, gyro.getYaw() + 5))
        )
    }
}
