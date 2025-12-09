package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Intake {
    private final DcMotorEx intake;
    public  final DcMotorEx belt;

    // This is still here because we might have to adjust the belt speed
    // Intake belt could easily get ripped off mid-game due to physical factors
    public final double beltForwardPower = 1.0;

    public Intake(HardwareMap hardwareMap)
    {
        belt    = hardwareMap.get(DcMotorEx.class, "Belt");
        intake  = hardwareMap.get(DcMotorEx.class, "Intake");

        // Lets just pray this will help our intake
        MotorConfigurationType configBelt = belt.getMotorType().clone();
        configBelt.setAchieveableMaxRPMFraction(1.0);
        belt.setMotorType(configBelt);

        // Just crank up the power pls
        MotorConfigurationType configIntake = intake.getMotorType().clone();
        configIntake.setAchieveableMaxRPMFraction(1.0);
        intake.setMotorType(configIntake);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2)
    {
        if(gamepad1.aWasPressed() || gamepad2.aWasPressed())
        {
            belt.setPower(beltForwardPower);
            intake.setPower(-1.0);
        }

        if(gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            belt.setPower(0.0);
            intake.setPower(0.0);
        }

        // For the second driver to only activate the belt
        // This should useful because intake + belt together take crap ton of voltage
        if(gamepad1.xWasPressed() || gamepad2.xWasPressed())
        {
            belt.setPower(beltForwardPower);
        }
    }

    public void start()
    {
        belt.setPower(beltForwardPower);
        intake.setPower(-1.0);
    }

    public void stop()
    {
        belt.setPower(0);
        intake.setPower(0.0);
    }

    public void setBeltSpeed(double speed)
    {
        belt.setPower(speed);
    }

    public void setIntakeSpeed(double speed)
    {
        intake.setPower(-speed);
    }
}
