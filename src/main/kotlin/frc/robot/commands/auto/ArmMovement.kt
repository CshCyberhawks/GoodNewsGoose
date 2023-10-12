package frc.robot.commands.auto

import cshcyberhawks.swolib.math.MiscCalculations
import frc.robot.subsystems.ArmSystem

interface GenericArmMovement {
    var isRunning: Boolean

    fun run()

    fun isDone(): Boolean
}

class AngleMovement(private val subsystem: ArmSystem, private val angle: Double) : GenericArmMovement {
    override var isRunning: Boolean = false

    override fun run() {
        subsystem.desiredArmAngle = angle
        isRunning = true
    }

    override fun isDone(): Boolean = subsystem.isFinished()
}

class ExtensionMovement(private val subsystem: ArmSystem, private val extensionPosition: Double) : GenericArmMovement {
    override var isRunning: Boolean = false

    override fun run() {
        subsystem.desiredExtensionPosition = extensionPosition
        isRunning = true
    }

    override fun isDone(): Boolean = subsystem.isFinished()
}

class TiltMovement(private val subsystem: ArmSystem, private val tiltPosition: Boolean, private val timer: Boolean = false) : GenericArmMovement {
    override var isRunning: Boolean = false
    var startTime = 0.0

    override fun run() {
        subsystem.desiredTilt = tiltPosition
        startTime = MiscCalculations.getCurrentTime()
        isRunning = true
    }

    override fun isDone(): Boolean = !timer || startTime + 5.0 < MiscCalculations.getCurrentTime()
}