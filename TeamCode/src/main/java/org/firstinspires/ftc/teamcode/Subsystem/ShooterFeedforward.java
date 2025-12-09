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

public class ShooterFeedforward {
    private final CachingDcMotorEx motorRight;
    private final CachingDcMotorEx motorLeft;
    private final CachingServo gate;

    private final List<VoltageSensor> voltageSensor;

    // This is the default shooting position for Auto.
    // Robot is supposed to be in the middle of a White Line of bigger shooting area
    public static final double MID_LINE_VELOCITY = 1170;

    private double targetVelocity = 0;

    // kV = 1 / MAX_RPM, and adjusted for kS

    public double kSRight = 0.045, kSLeft = 0.031, kV = 0.00035, kP = 0.00075;

    public ShooterFeedforward(HardwareMap hardwareMap)
    {
        motorRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "launchR"));
        motorLeft  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "launchL"));
        gate       = new CachingServo(hardwareMap.get(Servo.class, "Gate"));

        voltageSensor = hardwareMap.getAll(VoltageSensor.class);

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

        if(gamepad.right_trigger > 0.2)
        {
            gateOpen();
        } else if(gamepad.left_trigger > 0.2) {
            gateClose();
        }

        if(gamepad.rightBumperWasPressed())
        {
            setVelocity(targetVelocity + 20);
        } else if(gamepad.leftBumperWasPressed()){
            setVelocity(targetVelocity - 20);
        }
    }

    public void updateFeedforward()
    {
        // Find the minimal voltage across all voltage sensors
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensor) {
            double sensorVoltage = sensor.getVoltage();
            if (sensorVoltage > 0) {
                minVoltage = Math.min(minVoltage, sensorVoltage);
            }
        }

        double voltageCompenstation = minVoltage / 13.48;

        motorLeft.setPower(-1*(((kV / voltageCompenstation) * targetVelocity) + (kP * (targetVelocity - motorRight.getVelocity())) + kSLeft));
        motorRight.setPower((((kV / voltageCompenstation) * targetVelocity) + (kP * (targetVelocity - motorRight.getVelocity())) + kSRight));
    }

    public void updateTelemetry(TelemetryManager telemetry)
    {
        telemetry.addData("Shooter Left Velocity", motorLeft.getVelocity());
        telemetry.addData("Shooter Right Velocity", motorRight.getVelocity());
        telemetry.addData("Shooter Left Target", -targetVelocity);
        telemetry.addData("Shooter Right Target", targetVelocity);
    }


    private void setPower(double power)
    {
        motorRight.setPower(power);
        motorLeft.setPower(-power);
    }

    public double regressionVelocity(double distanceToGoal)
    {
        return 3.509 * distanceToGoal + 923.0;
    }

    public void setVelocity(double target)
    {
        targetVelocity = target;
    }

    public void gateClose() {
        gate.setPosition(0.35);
    }

    public void gateOpen() {
        gate.setPosition(0.5);
    }
}
