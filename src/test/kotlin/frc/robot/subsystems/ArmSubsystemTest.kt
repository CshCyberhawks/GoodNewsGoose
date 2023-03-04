package frc.robot.subsystems

import cshcyberhawks.swolib.hardware.implementations.TestGyro
import org.junit.jupiter.api.Test

import org.junit.jupiter.api.Assertions.*

class ArmSubsystemTest {
    companion object {
        val armSystem = ArmSubsystem(TestGyro())
    }

    @Test
    fun getRelativePositions() {
        val out1 = armSystem.getRelativePositions(129.58458665202514, 322.254248123196, 1.3098264776854747)
        assertEquals(0.9010118226332574, out1.x, 0.1)
        assertEquals(-1.0897329445161354, out1.y, 0.1)
        assertEquals(1.8264605558643126, out1.z, 0.1)

        val out2 = armSystem.getRelativePositions(322.48195755874974, 170.35355920733736, 4.6026734213159495)
        assertEquals(0.7446424313037315, out2.x, 0.1)
        assertEquals(-0.5717568761926496, out2.y, 0.1)
        assertEquals(-5.523454642698595, out2.z, 0.1)

        val out3 = armSystem.getRelativePositions(15.0599640243004, 84.96104622201517, 4.93281726581368)
        assertEquals(5.706909684585325, out3.x, 0.1)
        assertEquals(1.535565136793504, out3.y, 0.1)
        assertEquals(0.5210971811356134, out3.z, 0.1)

        val out4 = armSystem.getRelativePositions(296.0783703534486, 101.50733764511236, 3.988278431900587)
        assertEquals(2.148769750545371, out4.x, 0.1)
        assertEquals(-4.39037914275279, out4.y, 0.1)
        assertEquals(-0.9951287636215934, out4.z, 0.1)
    }

    @Test
    fun setRelativeArmPosition() {
    }
}