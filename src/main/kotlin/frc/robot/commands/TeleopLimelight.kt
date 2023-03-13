package frc.robot.commands

import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase

class TeleopLimelight(
        val limelight: Limelight,
        private val swerveDriveTrain: SwerveDriveTrain,
        private val pip: Int
) : CommandBase() {

    override fun initialize() {
        SmartDashboard.putBoolean("teleop ll running", true)
    }

    override fun isFinished(): Boolean {
        SmartDashboard.putBoolean("teleop ll running", true)
        return false
    }

    override fun end(interuppted: Boolean) {
        SmartDashboard.putBoolean("teleop ll running", false)
    }

    override fun execute() {
        val inputTwist =
                MiscCalculations.calculateDeadzone(limelight.getHorizontalOffset(), .5) / 32

        // if (Robot.pipIndex == pip) {
        swerveDriveTrain.drive(Vector2(0.0, .3), inputTwist, true)
        // }
    }
}
