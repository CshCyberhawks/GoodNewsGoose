package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import cshcyberhawks.swolib.autonomous.*;
import cshcyberhawks.swolib.autonomous.commands.*;
import cshcyberhawks.swolib.math.Vector2

//define an empty SequentialCommandGroup
public class TestingAuto(val swerveAuto: SwerveAuto) : SequentialCommandGroup() {

    //define the constructor
    init {
        //add the commands to the SequentialCommandGroup
        addCommands(
            GoToPosition(swerveAuto, Vector2(1.0, 0.0)),
        );
    }
}
