package frc.robot.commands

import cshcyberhawks.swolib.autonomous.*
import cshcyberhawks.swolib.autonomous.commands.*
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

// define an empty SequentialCommandGroup
public class TestingAuto(val swerveAuto: SwerveAuto, val gyro: GenericGyro) : SequentialCommandGroup() {

    // define the constructor
    init {
        gyro.setYawOffset()
        // add the commands to the SequentialCommandGroup
        addCommands(
                GoToPosition(swerveAuto, Vector2(0.0, 1.0)),
        )
    }
}
