package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

// Deprecated Shooter Subsystem
public class ShooterFeedback {
    private final DcMotorEx motorRight;
    private final DcMotorEx motorLeft;

    private final Servo gate;

    public static final double midLineVelocity = 1180;
    public static final double vertexVelocity  = 1450;

    public ShooterFeedback(HardwareMap hardwareMap)
    {
        gate = hardwareMap.get(Servo.class, "Gate");

        motorRight = hardwareMap.get(DcMotorEx.class, "launchR");
        motorLeft  = hardwareMap.get(DcMotorEx.class, "launchL");

        PIDFCoefficients coefficientsLeft  = new PIDFCoefficients(10, 0.0, 0, 13.76, MotorControlAlgorithm.PIDF);
        PIDFCoefficients coefficientsRight = new PIDFCoefficients(10, 0.0, 0, 13.76, MotorControlAlgorithm.PIDF);

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

    public void update(Gamepad gamepad, DcMotorEx belt)
    {
        // Unstuck the ball
        if(gamepad.rightStickButtonWasPressed())
        {
            setVelocity(-2300);
            belt.setPower(-1.0);
        } else if(gamepad.leftStickButtonWasPressed()) {
            setVelocity(0);
            belt.setPower(0);
        }

        if(gamepad.yWasPressed()) {
            setVelocity(0);
        }

        if(gamepad.dpadUpWasPressed())
        {
            setVelocity(midLineVelocity);
        } else if(gamepad.dpadDownWasPressed()) {
            setVelocity(vertexVelocity);
        }

        if(gamepad.leftBumperWasPressed()) {
            setVelocity(motorRight.getVelocity() - 100);
        } else if(gamepad.rightBumperWasPressed()) {
            setVelocity(motorRight.getVelocity() + 100);
        }

        if(gamepad.right_trigger > 0.2)
        {
            gateOpen();
        } else if(gamepad.left_trigger > 0.2) {
            gateClose();
        }
    }

    public void sendTelemetry(TelemetryManager telemetry)
    {
        telemetry.addData("Shooter Left Velocity", motorLeft.getVelocity());
        telemetry.addData("Shooter Right Velocity", motorRight.getVelocity());
        telemetry.addData("Right Shooter Current", motorRight.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Left Shooter Current", motorLeft.getCurrent(CurrentUnit.MILLIAMPS));
    }

    public void setVelocity(double velocity)
    {
        motorRight.setVelocity(velocity + 37);
        motorLeft.setVelocity(-velocity);
    }

    public void gateClose() {
        gate.setPosition(0.0);
    }

    public void gateOpen() {
        gate.setPosition(0.25);
    }
}
