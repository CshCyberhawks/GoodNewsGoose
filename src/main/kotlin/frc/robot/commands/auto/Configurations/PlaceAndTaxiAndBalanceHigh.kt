package frc.robot.commands.auto.Configurations

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.arm.AutoPlaceHigh
import frc.robot.commands.auto.arm.AutoPlaceMid
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawSystem

class PlaceAndTaxiAndBalanceHigh(private val swerveAuto: SwerveAuto, private val gyro: GenericGyro, private val armSystem: ArmSystem, private val autoPathManager: AutoPathManager, private val swerveSystem: SwerveDriveTrain) :
        SequentialCommandGroup() {

    // define the constructor
    init {
//        gyro.setYawOffset()
        // add the commands to the SequentialCommandGroup
        addCommands(
                AutoPlaceMid(armSystem),
                autoPathManager.paths["TaxiAndBalance"]!!,
        )
    }
}
