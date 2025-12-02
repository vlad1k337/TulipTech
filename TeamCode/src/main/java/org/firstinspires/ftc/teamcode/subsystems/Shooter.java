package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    private final DcMotorEx motorRight;
    private final DcMotorEx motorLeft;

    private final Servo gate;

    // This is the default shooting position for Auto.
    // Robot is supposed to be in the middle of a White Line of bigger shooting area
    public static final double midLineVelocity = 1140;

    public Shooter(HardwareMap hardwareMap)
    {
        gate = hardwareMap.get(Servo.class, "Gate");

        motorRight = hardwareMap.get(DcMotorEx.class, "launchR");
        motorLeft  = hardwareMap.get(DcMotorEx.class, "launchL");

        PIDFCoefficients coefficientsLeft  = new PIDFCoefficients(10, 3.0, 0, 8.5);
        PIDFCoefficients coefficientsRight = new PIDFCoefficients(10, 3.0, 0, 6.75);

        MotorConfigurationType configR = motorRight.getMotorType().clone();
        configR.setAchieveableMaxRPMFraction(1.0);
        motorRight.setMotorType(configR);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsRight);

        MotorConfigurationType configL = motorLeft.getMotorType().clone();
        configL.setAchieveableMaxRPMFraction(1.0);
        motorLeft.setMotorType(configL);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsLeft);
    }

    public void update(Gamepad gamepad)
    {
        if(gamepad.yWasPressed()) {
            setVelocity(0);
        }

        if(gamepad.dpadUpWasPressed())
        {
            setVelocity(midLineVelocity);
        }

        if(gamepad.leftBumperWasPressed()) {
            setVelocity(motorRight.getVelocity() - 300);
        } else if(gamepad.rightBumperWasPressed()) {
            setVelocity(motorRight.getVelocity() + 300);
        }

        if(gamepad.right_trigger > 0.0)
        {
            gateOpen();
        } else if(gamepad.left_trigger > 0.0) {
            gateClose();
        }
    }

    public void sendTelemetry(TelemetryManager telemetry)
    {
        telemetry.addData("Shooter Velocity", motorRight.getVelocity());
        telemetry.addData("Right Shooter Current", motorRight.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Left Shooter  Current", motorLeft.getCurrent(CurrentUnit.MILLIAMPS));
    }

    public void setPower(double power)
    {
        motorRight.setPower(power);
        motorLeft.setPower(-power);
    }

    public void setVelocity(double velocity)
    {
        motorRight.setVelocity(velocity);
        motorLeft.setVelocity(-velocity);
    }

    public void gateClose() {
        gate.setPosition(0.0);
    }

    public void gateOpen() {
        gate.setPosition(0.3);
    }
}
