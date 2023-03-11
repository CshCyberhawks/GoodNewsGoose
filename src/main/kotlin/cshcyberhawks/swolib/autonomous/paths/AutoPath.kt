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
    private val commandsIn: HashMap<Int, Pair<CommandBase, AttachedCommandType>> = HashMap()
) : CommandBase() {
    private var positions: List<FieldPosition>

    private var currentCommand: CommandBase? = null
    private var attachedCommand: CommandBase? = null
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
            if (attachedCommand != null && attachedCommand?.isFinished == false) {
                attachedCommand?.cancel()
                attachedCommand = null
            }

            if (commandsIn.containsKey(currentIndex) && commandsIn[currentIndex] != null) {
                val (cmd, type) = commandsIn[currentIndex]!!

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
