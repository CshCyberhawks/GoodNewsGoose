package frc.robot.commands

import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.Timer

class HardwareTestCommand(private val swerveDriveTrain: SwerveDriveTrain) : CommandBase() {
    init {
        addRequirements(swerveDriveTrain)
    }

    override fun execute() {
        swerveDriveTrain.drive(Vector2(0.0, 1.0), 0.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(0.0, -1.0), 0.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(1.0, 0.0), 0.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(-1.0, 0.0), 0.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(), 1.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(), -1.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(1.0, 1.0), 0.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(-1.0, -1.0), 0.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(1.0, 0.0), 1.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(0.0, 1.0), 1.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(1.0, 1.0), 1.0)
        Timer.delay(1.0)
        swerveDriveTrain.drive(Vector2(1.0, 1.0), -1.0)
    }
}
