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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    private final DcMotorEx motorRight;
    private final DcMotorEx motorLeft;

    private boolean isStuck = false;

    private final Servo gate;

    // This is the default shooting position for Auto.
    // Robot is supposed to be in the middle of a White Line of bigger shooting area
    public static final double midLineVelocity = 1250;
    public static final double vertexVelocity  = 1450;

    public Shooter(HardwareMap hardwareMap)
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
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsRight);

        MotorConfigurationType configL = motorLeft.getMotorType().clone();
        configL.setAchieveableMaxRPMFraction(1.0);
        motorLeft.setMotorType(configL);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficientsLeft);
    }

    public void update(Gamepad gamepad, DcMotorEx belt)
    {
        double averageCurrent = (motorLeft.getCurrent(CurrentUnit.MILLIAMPS) + motorRight.getCurrent(CurrentUnit.MILLIAMPS))/2;
        ElapsedTime unstuckTimer = new ElapsedTime();
        unstuckTimer.reset();
        while(averageCurrent > 8000)
        {
            setVelocity(-2400);
            belt.setPower(-1.0);

            if(unstuckTimer.milliseconds() > 500)
            {
                setVelocity(0);
                belt.setPower(0);

                break;
            }
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

        if(gamepad.right_trigger > 0.0)
        {
            gateOpen();
        } else if(gamepad.left_trigger > 0.0) {
            gateClose();
        }
    }

    public void sendTelemetry(TelemetryManager telemetry)
    {
        telemetry.addData("Shooter Left Velocity", motorLeft.getVelocity());
        telemetry.addData("Shooter Right Velocity", motorRight.getVelocity());
        telemetry.addData("Right Shooter Current", motorRight.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Left ShooterCurrent", motorLeft.getCurrent(CurrentUnit.MILLIAMPS));
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
