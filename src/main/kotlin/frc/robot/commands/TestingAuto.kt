package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.FieldPosition
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

// define an empty SequentialCommandGroup
class TestingAuto(val swerveAuto: SwerveAuto, val gyro: GenericGyro) : SequentialCommandGroup() {

    // define the constructor
    init {
        gyro.setYawOffset()
        // add the commands to the SequentialCommandGroup
        addCommands(
            GoToPosition(swerveAuto, FieldPosition(0.0, 2.0, 0.0)),
             GoToPosition(swerveAuto, FieldPosition(0.0, 2.0, 180.0)),
             GoToPosition(swerveAuto, FieldPosition(0.0, 0.0, 0.0)),
//             GoToPosition(swerveAuto, FieldPosition(-1.0, 2.0, 90.0)),
//            GoToPosition(swerveAuto, FieldPosition(5.0, 0.0, 90.0))

        )
    }
}
