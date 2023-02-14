package frc.robot.constants

object MotorPorts {
    // TalonSRX Motors
    const val frontRightTurnMotor: Int = 8
    const val frontLeftTurnMotor: Int = 7
    const val backRightTurnMotor: Int = 9
    const val backLeftTurnMotor: Int = 6

    // Falcon Motors
    const val frontRightDriveMotor: Int = 4
    const val frontLeftDriveMotor: Int = 3
    const val backRightDriveMotor: Int = 5
    const val backLeftDriveMotor: Int = 2

    // Encoders
    const val frontRightEncoder = 0
    const val frontLeftEncoder = 1
    const val backRightEncoder = 3
    const val backLeftEncoder = 2

    val turnEncoderOffsets: Array<Double> = arrayOf(345.1904697418213, 233.98533630371094, 357.19047808647156, 347.40471935272217)
}