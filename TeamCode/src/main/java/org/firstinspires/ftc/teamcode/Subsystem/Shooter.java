package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Shooter {
    private final CachingDcMotorEx motorRight;
    private final CachingDcMotorEx motorLeft;
    private final CachingServo gate;

    // Velocity required to shoot from the middle of a white line.
    // Decrease it if robot is overshooting, increase if it's undershooting..
    public static final double MID_LINE_VELOCITY = 1320;

    private final double GATE_OPEN   = 0.0;
    private final double GATE_CLOSED = 0.6;

    private double targetVelocity = 0;

    // kV = 1 / MAX_RPM, and adjusted for kS
    public double kSRight = 0.041, kSLeft = 0.031, kV = 0.00035, kP = 0.001;

    public Shooter(HardwareMap hardwareMap)
    {
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "launchR"));
        motorLeft  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "launchL"));
        gate       = new CachingServo(hardwareMap.get(Servo.class, "Gate"));
        gate.setDirection(Servo.Direction.REVERSE);

        MotorConfigurationType configR = motorRight.getMotorType().clone();
        configR.setAchieveableMaxRPMFraction(1.0);
        motorRight.setMotorType(configR);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType configL = motorLeft.getMotorType().clone();
        configL.setAchieveableMaxRPMFraction(1.0);
        motorLeft.setMotorType(configL);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(Gamepad gamepad)
    {
        updateFeedforward();
        updateControls(gamepad);
    }

    private void updateControls(Gamepad gamepad)
    {
        if(gamepad.yWasPressed()) {
            setVelocity(0);
        }

        if(gamepad.dpadUpWasPressed())
        {
            setVelocity(MID_LINE_VELOCITY);
        }

        if(gamepad.rightBumperWasPressed())
        {
            setVelocity(targetVelocity + 20);
        } else if(gamepad.leftBumperWasPressed()){
            setVelocity(targetVelocity - 20);
        }

        if(gamepad.right_trigger > 0.2)
        {
            gateOpen();
        } else if(gamepad.left_trigger > 0.2) {
            gateClose();
        }
    }

    public void updateFeedforward()
    {
        motorLeft.setPower(-1*((kV * targetVelocity) + (kP * (targetVelocity + motorLeft.getVelocity())) + kSLeft));
        motorRight.setPower((kV * targetVelocity) + (kP * (targetVelocity - motorRight.getVelocity())) + kSRight);
    }

    public void updateTelemetry(TelemetryManager telemetry)
    {
        telemetry.addData("Shooter Left Velocity", motorLeft.getVelocity());
        telemetry.addData("Shooter Right Velocity", motorRight.getVelocity());
        telemetry.addData("Shooter Left Target", -targetVelocity);
        telemetry.addData("Shooter Right Target", targetVelocity);
    }

    public void setVelocity(double target)
    {
        targetVelocity = target;
    }

    public void gateClose() {
        gate.setPosition(GATE_CLOSED);
    }

    public void gateOpen() {
        gate.setPosition(GATE_OPEN);
    }
}
