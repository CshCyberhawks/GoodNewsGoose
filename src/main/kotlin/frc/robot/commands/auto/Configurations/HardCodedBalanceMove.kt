package frc.robot.commands.auto.Configurations

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.Vector2
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

public class HardCodedBalanceMove(swerveAuto: SwerveAuto) : SequentialCommandGroup() {
    init {
        addCommands(
                GoToPosition(swerveAuto, Vector2(0.0, -2.55)),
                GoToPosition(swerveAuto, FieldPosition(0.0, -2.55, 15.0))
        )
    }
}