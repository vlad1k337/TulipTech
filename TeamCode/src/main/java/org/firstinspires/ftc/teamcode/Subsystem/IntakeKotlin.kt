package org.firstinspires.ftc.teamcode.Subsystem

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx

class IntakeKotlin(val hardwareMap: HardwareMap) {
    private val belt: CachingDcMotorEx = CachingDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "Belt"))
    private val intake: CachingDcMotorEx = CachingDcMotorEx(hardwareMap.get(DcMotorEx::class.java, "Intake"))

    private val beltForwardPower = 1.0

    init {
        val configBelt: MotorConfigurationType = belt.motorType
        configBelt.maxRPM = 1.0
        belt.motorType = configBelt

        val configIntake = intake.motorType
        configIntake.maxRPM = 1.0
        intake.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun update(gamepad1: Gamepad, gamepad2: Gamepad)
    {
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            belt.power = beltForwardPower
            intake.power = -1.0
        }

        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            belt.power = 0.0
            intake.power = 0.0
        }

        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            belt.power = beltForwardPower
        }
    }

    fun start() {
        belt.power = beltForwardPower
        intake.power = -1.0
    }

    fun stop() {
        belt.power = 0.0
        intake.power = 0.0
    }

    fun setBeltSpeed(speed: Double) {
        belt.power = speed
    }
}