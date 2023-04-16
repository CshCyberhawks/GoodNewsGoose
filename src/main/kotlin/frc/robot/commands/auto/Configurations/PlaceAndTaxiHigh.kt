package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import frc.robot.commands.auto.AutoPlaceHigh
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawSystem

// define an empty SequentialCommandGroup
class PlaceAndTaxiHigh(private val swerveAuto: SwerveAuto, private val gyro: GenericGyro, private val armSystem: ArmSystem, private val autoPathManager: AutoPathManager, private val swerveSystem: SwerveDriveTrain, private val clawSystem: ClawSystem) :
        SequentialCommandGroup() {

    // define the constructor
    init {
        gyro.setYawOffset()
        // add the commands to the SequentialCommandGroup
        addCommands(
                AutoPlaceHigh(armSystem, clawSystem),
                autoPathManager.paths["TaxiTop"]!!,
        )
    }
}
