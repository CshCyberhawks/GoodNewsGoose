package cshcyberhawks.swolib.autonomous.paths

import com.beust.klaxon.Klaxon
import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.limelight.AttachedCommandType
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.math.Vector3
import edu.wpi.first.wpilibj2.command.CommandBase
import java.io.File

class AutoPath(
    inputFile: File,
    val swerveAuto: SwerveAuto,
    val gyro: GenericGyro,
    val commandsInput: HashMap<Int, Pair<CommandBase, AttachedCommandType>> = HashMap()
) : CommandBase() {
    val positions =
        Klaxon().parseArray<AutoPathNode>(inputFile)!!.map { Vector2(
            -it.point.y,
            it.point.x
        )
        }

    var currentCommand: CommandBase? = null
    var attachedCommand: CommandBase? = null
    var currentIndex = 1

    init {


        // if (commandsIn.size != 0) {
        //     commandsIn.forEach { (key, (cmd, typ)) ->
        //         if (key < commandsToRun.size) commandsToRun.add(cmd, key)
        //     }
        // }
        //        gyro.setYawOffset(jsonData.startPosition.angle)
    }

    override fun initialize() {
        swerveAuto.swo.fieldPosition = Vector3(positions[0].x, positions[0].y, 0.0)
    }

    override fun execute() {
        if ((currentCommand == null || currentCommand?.isFinished == true) && currentIndex < positions.size) {
            if (attachedCommand != null && attachedCommand?.isFinished == false) {
                attachedCommand?.cancel()
                attachedCommand = null
            }

            if (commandsInput.containsKey(currentIndex) && commandsInput[currentIndex] != null) {
                val (cmd, type) = commandsInput[currentIndex]!!

                when (type) {
                    AttachedCommandType.ASYNC -> {
                        cmd.schedule()
                    }
                    AttachedCommandType.SYNC -> {
                        attachedCommand = cmd
                        attachedCommand?.schedule()
                    }
                }
            }

            currentCommand = GoToPosition(swerveAuto, positions[currentIndex++])
            currentCommand?.schedule()
        }
    }

    override fun isFinished(): Boolean = currentIndex == positions.size
}
