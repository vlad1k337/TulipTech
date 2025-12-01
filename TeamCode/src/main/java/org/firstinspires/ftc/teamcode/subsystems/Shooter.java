package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Shooter {
    private DcMotorEx motorRight;
    private DcMotorEx motorLeft;

    private Servo gate;
    private double gatePosition;

    private String shootingMode = "NOT SET";

    // This is the default shooting position for Auto.
    // Robot is supposed to be in the middle of a White Line of bigger shooting area
    public static final double midLineVelocity = 1120;

    public static final double midLinePower = 0.41;

    public Shooter(HardwareMap hardwareMap)
    {
        motorRight = hardwareMap.get(DcMotorEx.class, "launchR");
        motorLeft  = hardwareMap.get(DcMotorEx.class, "launchL");

        MotorConfigurationType configR = motorRight.getMotorType().clone();
        configR.setAchieveableMaxRPMFraction(1.0);
        motorRight.setMotorType(configR);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorConfigurationType configL = motorLeft.getMotorType().clone();
        configL.setAchieveableMaxRPMFraction(1.0);
        motorLeft.setMotorType(configL);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gate = hardwareMap.get(Servo.class, "Gate");
    }

    public void update(Gamepad gamepad)
    {
        if(gamepad.yWasPressed()) {
            setVelocity(0);
            shootingMode = "Not Set";
        }

        if(gamepad.dpadUpWasPressed())
        {
            setVelocity(midLineVelocity);
            shootingMode = "Mid Line";
        }

        if(gamepad.leftBumperWasPressed()) {
            setVelocity(motorRight.getVelocity() - 300);
        } else if(gamepad.rightBumperWasPressed()) {
            setVelocity(motorRight.getVelocity() + 300);
        }

        if(gamepad.right_trigger > 0.0)
        {
            gatePosition = 0.3;
        } else if(gamepad.left_trigger > 0.0) {
            gatePosition = 0.0;
        }

        gate.setPosition(gatePosition);
    }

    public void sendTelemetry(TelemetryManager telemetry)
    {
        telemetry.debug("Shooter Mode:  " + shootingMode);
        telemetry.debug("Shooter Velocity: " + motorRight.getVelocity());
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
