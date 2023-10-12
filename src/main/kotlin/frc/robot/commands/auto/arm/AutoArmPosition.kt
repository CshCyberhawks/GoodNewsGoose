package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.commands.auto.GenericArmMovement
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem
import java.util.*

/**
 * @property armSystem
 */
class AutoArmPosition(private val armSystem: ArmSystem, armMovementQueue: List<GenericArmMovement>) : CommandBase() {
    /**
     * Creates a new ExampleCommand.
     */
    private val armQueue = arrayListOf<GenericArmMovement>()

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        //        addRequirements(armSystem)
        for (armMovement in armMovementQueue) {
            armQueue.add(armMovement)
        }
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    private fun armLogic() {
        if (armQueue.size != 0) {
            if (!armQueue[0].isRunning) {
                armQueue[0].run()
            }

            if (armQueue[0].isDone()) {
                armQueue.removeAt(0)
            }

            return
        }
    }

    override fun execute() {
        armLogic()

        armSystem.run()
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
//        armSystem.kill()
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return armQueue.size == 0
    }
}
