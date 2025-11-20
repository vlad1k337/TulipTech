package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.bylazar.telemetry.TelemetryManager;
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

    private String shootingMode = "NOT SET";
    private double launchSpeed = 0.0;

    // This is the default shooting position for Auto.
    // Robot is supposed to be in the middle of a White Line in Big Shooting Area
    final double midLinePower = 0.41;

    public Shooter(HardwareMap hardwareMap)
    {
        motorRight = hardwareMap.get(DcMotorEx.class, "launchR");
        motorLeft  = hardwareMap.get(DcMotorEx.class, "launchL");

        MotorConfigurationType configR = motorRight.getMotorType().clone();
        configR.setAchieveableMaxRPMFraction(1.0);
        motorRight.setMotorType(configR);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType configL = motorLeft.getMotorType().clone();
        configL.setAchieveableMaxRPMFraction(1.0);
        motorLeft.setMotorType(configL);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gate = hardwareMap.get(Servo.class, "Gate");
    }

    public void update(Gamepad gamepad)
    {
        if(gamepad.yWasPressed()) {
            launchSpeed = 0.0;
            shootingMode = "NOT SET";
        }

        if(gamepad.dpadUpWasPressed())
        {
            launchSpeed  = midLinePower;
            shootingMode = "Mid Line";
        }

        if(gamepad.leftBumperWasPressed()) {
            if (launchSpeed - 0.1 > 0.0) launchSpeed -= 0.1;
        } else if(gamepad.rightBumperWasPressed()) {
            if(launchSpeed + 0.1 < 1.0) launchSpeed += 0.1;
        }

        if(gamepad.right_trigger > 0.0)
        {
            gate.setPosition(0.1);
        } else if(gamepad.left_trigger > 0.0) {
            gate.setPosition(0.0);
        }

        motorRight.setPower(launchSpeed);
        motorLeft.setPower(-launchSpeed);
    }

    public void sendTelemetry(TelemetryManager telemetry)
    {
        telemetry.debug("Shooter Power: " + launchSpeed);
        telemetry.debug("Shooter Mode:  " + shootingMode);
        telemetry.debug("Shooter Velocity: " + motorRight.getVelocity());
    }
}
