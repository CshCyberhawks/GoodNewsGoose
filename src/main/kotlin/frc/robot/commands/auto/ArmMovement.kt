package frc.robot.commands.auto

import cshcyberhawks.swolib.math.MiscCalculations
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

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

class TiltMovement(private val subsystem: ArmSystem, private val tiltPosition: Boolean) : GenericArmMovement {
    override var isRunning: Boolean = false
    private var startTime = 0.0
    private var timer = false

    override fun run() {
        if (subsystem.desiredTilt != tiltPosition && !tiltPosition) {
            timer = true
        }

        subsystem.desiredTilt = tiltPosition
        startTime = MiscCalculations.getCurrentTimeSeconds()
        isRunning = true
    }

    override fun isDone(): Boolean = !timer || startTime + 0.5 < MiscCalculations.getCurrentTimeSeconds()
}

class ClawAction(private val state: ClawState) : GenericArmMovement {
    override var isRunning: Boolean = false

    override fun run() {
        ClawSystem.clawState = state
        isRunning = true
    }

    override fun isDone(): Boolean = true
}