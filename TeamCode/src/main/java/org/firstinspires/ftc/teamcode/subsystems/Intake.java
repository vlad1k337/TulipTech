package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Intake {
    private DcMotorEx intake;
    private DcMotor belt;

    public final double beltForwardSpeed = 1.0;
    public final double beltReverseSpeed = -0.41;

    private final double intakeSpeed = 1.0;

    public Intake(HardwareMap hardwareMap)
    {
        belt    = hardwareMap.get(DcMotor.class, "Belt");
        intake  = hardwareMap.get(DcMotorEx.class, "Intake");

        MotorConfigurationType configIntake = intake.getMotorType().clone();
        configIntake.setAchieveableMaxRPMFraction(1.0);
        intake.setMotorType(configIntake);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update(Gamepad gamepad)
    {
        if(gamepad.aWasPressed())
        {
            belt.setPower(beltForwardSpeed);
            intake.setPower(-intakeSpeed);
        } else if(gamepad.bWasPressed()) {
            belt.setPower(0.0);
            intake.setPower(0.0);
        }
    }

    public void start()
    {
        belt.setPower(beltForwardSpeed);
        intake.setPower(-intakeSpeed);
    }

    public void stop()
    {
        belt.setPower(0);
        intake.setPower(0);
    }

    public void setBeltSpeed(double speed)
    {
        belt.setPower(speed);
    }
}
