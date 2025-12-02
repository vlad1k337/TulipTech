package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Intake {
    private final DcMotorEx intake;
    private final DcMotorEx belt;

    public final double beltForwardPower = 1.0;
    // Ticks Per Second
    private final double intakeSpeed = 2520;

    public Intake(HardwareMap hardwareMap)
    {
        belt    = hardwareMap.get(DcMotorEx.class, "Belt");
        intake  = hardwareMap.get(DcMotorEx.class, "Intake");

        // Lets just pray this will help our intake
        MotorConfigurationType configBelt = belt.getMotorType().clone();
        configBelt.setAchieveableMaxRPMFraction(1.0);
        belt.setMotorType(configBelt);

        MotorConfigurationType configIntake = intake.getMotorType().clone();
        configIntake.setAchieveableMaxRPMFraction(1.0);
        intake.setMotorType(configIntake);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Gamepad gamepad)
    {
        if(gamepad.aWasPressed())
        {
            belt.setPower(beltForwardPower);
            intake.setVelocity(-intakeSpeed);
        }

        if(gamepad.bWasPressed()) {
            belt.setPower(0.0);
            intake.setVelocity(0.0);
        }

        // For the second driver to only activate the belt
        // This should useful because intake + belt together take crap ton of voltage
        if(gamepad.xWasPressed())
        {
            belt.setPower(beltForwardPower);
        }
    }

    public void start()
    {
        belt.setPower(beltForwardPower);
        intake.setVelocity(-intakeSpeed);
    }

    public void stop()
    {
        belt.setPower(0);
        intake.setVelocity(0.0);
    }

    public void setBeltSpeed(double speed)
    {
        belt.setPower(speed);
    }

    public void setIntakeSpeed(double speed)
    {
        intake.setVelocity(speed);
    }
}
