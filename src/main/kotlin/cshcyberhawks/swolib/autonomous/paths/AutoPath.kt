package cshcyberhawks.swolib.autonomous.paths

import com.beust.klaxon.Klaxon
import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.AngleCalculations
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.Vector3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.CommandBase
import java.io.File

class AutoPath(
    inputFile: File,
    val swerveAuto: SwerveAuto,
    val gyro: GenericGyro,
    private val commandsList: HashMap<Int, Array<AttachedCommand>> = HashMap()
) : CommandBase() {
    private var positions: List<FieldPosition>

    private var currentCommand: CommandBase? = null
    private var attachedCommands: MutableList<CommandBase> = mutableListOf()
    private var currentIndex = 1

    init {
        if (DriverStation.getAlliance() == Alliance.Blue) {
            this.positions = Klaxon().parseArray<AutoPathNode>(inputFile)!!.map {
                FieldPosition(-it.point.y, it.point.x, AngleCalculations.wrapAroundAngles(it.point.angle))
            }
        } else {
            this.positions = Klaxon().parseArray<AutoPathNode>(inputFile)!!.map {
                FieldPosition(it.point.y, it.point.x, AngleCalculations.wrapAroundAngles(it.point.angle))
            }
        }

    }

    override fun initialize() {
        swerveAuto.swo.fieldPosition =
            Vector3(positions[0].x, positions[0].y, AngleCalculations.wrapAroundAngles(positions[0].angle + 180))
    }

    override fun execute() {
        if ((currentCommand == null || currentCommand?.isFinished == true) && currentIndex < positions.size) {
            for (cmd in attachedCommands) {
                if (!cmd.isFinished) {
                    cmd.cancel()
                }
            }
            attachedCommands = mutableListOf()

            if (commandsList.containsKey(currentIndex) && commandsList[currentIndex] != null) {
                val cmds = commandsList[currentIndex]!!

                for (cmd in cmds) {
                    when (cmd.attachedCommandType) {
                        AttachedCommandType.ASYNC -> {
                            cmd.command.schedule()
                        }

                        AttachedCommandType.SYNC -> {
                            attachedCommands.add(cmd.command)
                            cmd.command.schedule()
                        }
                    }
                }
            }

            currentCommand = GoToPosition(swerveAuto, positions[currentIndex++])
            currentCommand?.schedule()
        }
    }


    override fun isFinished(): Boolean = currentIndex == positions.size
}
