package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor belt, intake;

    private final double beltSpeed   = 0.75;
    private final double intakeSpeed = 1.0;

    public Intake(HardwareMap hardwareMap)
    {
        belt    = hardwareMap.get(DcMotor.class, "Belt");
        intake  = hardwareMap.get(DcMotor.class, "Intake");
    }

    public void update(Gamepad gamepad)
    {
        if(gamepad.aWasPressed())
        {
            belt.setPower(beltSpeed);
            intake.setPower(-intakeSpeed);
        } else if(gamepad.bWasPressed()) {
            belt.setPower(0.0);
            intake.setPower(0.0);
        }
    }

    public void start()
    {
        belt.setPower(beltSpeed);
        intake.setPower(-intakeSpeed);
    }

    public void stop()
    {
        belt.setPower(0);
        intake.setPower(0);
    }
}
