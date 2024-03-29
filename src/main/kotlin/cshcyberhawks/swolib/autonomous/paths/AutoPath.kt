package cshcyberhawks.swolib.autonomous.paths

import com.beust.klaxon.Klaxon
import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.AngleCalculations
import cshcyberhawks.swolib.math.FieldPosition
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import java.io.File

class AutoPath(
    inputFile: File,
    val swerveAuto: SwerveAuto,
    val gyro: GenericGyro,
    alliance: Alliance,
    private val commandsList: HashMap<Int, CommandBase> = HashMap()
) : CommandBase() {
    private var positions: MutableList<FieldPosition>

    private var currentCommand: CommandBase? = null
    private var attachedCommand: CommandBase? = null
    private var currentIndex = 0

    init {
        SmartDashboard.putString("Auto Path Alliance", alliance.name)

        if (alliance == Alliance.Blue) {
            this.positions = Klaxon().parseArray<AutoPathNode>(inputFile)!!.map {
                FieldPosition(-it.point.y, it.point.x, AngleCalculations.wrapAroundAngles(it.point.angle))
            }.toMutableList()
        } else {
            this.positions = Klaxon().parseArray<AutoPathNode>(inputFile)!!.map {
                FieldPosition(it.point.y, it.point.x, AngleCalculations.wrapAroundAngles(it.point.angle))
            }.toMutableList()
        }

        positions.add(positions.last())
    }

    override fun initialize() {
        gyro.setYawOffset(positions[0].angle)
//        swerveAuto.swo.fieldPosition =
//            Vector3(positions[0].x, positions[0].y, AngleCalculations.wrapAroundAngles(positions[0].angle + 180))
    }

    override fun execute() {
        if ((currentCommand == null || currentCommand?.isFinished == true) && (attachedCommand == null || attachedCommand?.isFinished == false) && currentIndex < positions.size) {
            attachedCommand = null
            currentCommand = null
            if (commandsList.containsKey(currentIndex) && commandsList[currentIndex] != null) {
                attachedCommand = commandsList[currentIndex]!!
                attachedCommand?.schedule()
            }

            val pos = positions[currentIndex++]
            currentCommand = GoToPosition(swerveAuto, FieldPosition(pos.x, pos.y, pos.angle))
            currentCommand?.schedule()
        }
    }


    override fun isFinished(): Boolean {
        SmartDashboard.putBoolean("auto path finished", currentIndex == positions.size && currentCommand != null && currentCommand?.isFinished == true)
        return currentIndex == positions.size && currentCommand != null && currentCommand?.isFinished == true
    }
}
