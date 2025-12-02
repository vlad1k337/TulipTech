package org.firstinspires.ftc.teamcode.utilmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class MotorFeedforwardTuner extends OpMode {
    private TelemetryManager telemetryM;
    private Follower follower;

    private final double targetVelocity = 1140;

    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double F = 0;

    private String motorName = "launchL";
    private DcMotorEx motor;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        motor  = hardwareMap.get(DcMotorEx.class, motorName);

        MotorConfigurationType configR = motor.getMotorType().clone();
        configR.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(configR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients coefficients = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        P = coefficients.p;
        I = coefficients.i;
        D = coefficients.d;
    }

    @Override
    public void start()
    {
        telemetryM.debug("This is motor feedforward tuner.");
        telemetryM.debug("Use right bumper to increase F by 0.25, and left bumper to decrease by 0.25");
        telemetryM.debug("Press X and Y to start and stop the desired motor. Observe the graph of a PID state on Panels");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        telemetryM.addData("Current", motor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetryM.addData("Target Velocity", targetVelocity);
        telemetryM.addData("Actual Velocity", motor.getVelocity());
        telemetryM.addData("PIDF coefficients", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        telemetryM.update(telemetry);

        if(gamepad1.rightBumperWasPressed())
        {
            F += 0.25;
        } else if(gamepad1.leftBumperWasPressed()) {
            F -= 0.25;
        }

        motor.setVelocityPIDFCoefficients(P, I, D, F);

        if(gamepad1.xWasPressed())
        {
            motor.setVelocity(targetVelocity);
        } else if(gamepad1.yWasPressed()) {
            motor.setVelocity(0);
        }
    }
}
