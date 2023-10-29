package cshcyberhawks.swolib.autonomous.paths

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.CommandBase

class AutoPathManager(
    swerveAuto: SwerveAuto,
    gyro: GenericGyro,
    alliance: Alliance,
    private val commandsToRun: HashMap<String, HashMap<Int, CommandBase>> =
        hashMapOf()
) {
    val paths: HashMap<String, AutoPath> = hashMapOf()

    init {
        Filesystem.getDeployDirectory().resolve("./paths/").listFiles()?.forEach {
            val name = it.name.substring(0, it.name.indexOf("."))
            if (commandsToRun.size != 0 && commandsToRun.containsKey(name)) {
                paths[name] = AutoPath(it, swerveAuto, gyro, alliance, commandsToRun[name]!!)
            } else {
                paths[name] = AutoPath(it, swerveAuto, gyro, alliance)
            }
        }
    }
}
