package frc.robot.commands

import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Robot

class TeleopLimelight(val limelight: Limelight, private val swerveDriveTrain: SwerveDriveTrain, private val pip: Int) :
    CommandBase() {
    override fun execute() {
        val inputTwist =
            MiscCalculations.calculateDeadzone(limelight.getHorizontalOffset(), .5) / 32

        if (Robot.pipIndex == pip) {
            swerveDriveTrain.drive(Vector2(1.0, 0.0), inputTwist)
        }
    }
}
