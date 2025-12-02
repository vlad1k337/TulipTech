package org.firstinspires.ftc.teamcode.utilmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class StickExpoTuner extends OpMode {
    private Follower follower;
    private TelemetryManager telemetryM;

    private double left_stick  = 1.7;
    private double right_stick = 1.5;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.update();
    }

    @Override
    public void start() {
        telemetryM.debug("Use left bumper/trigger to increase/decrease left stick exponential smoothing");
        telemetryM.debug("Same for the right one..");

        follower.startTeleopDrive();
        follower.update();
    }

    private double expo(double input, double exponent)
    {
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    private void updateDrive(Gamepad gamepad)
    {
        if(gamepad.leftBumperWasPressed())
        {
            left_stick += 0.1;
        } else if(gamepad.left_trigger > 0) {
            left_stick -= 0.1;
        }

        if(gamepad.rightBumperWasPressed())
        {
            right_stick += 0.1;
        } else if(gamepad.right_trigger > 0) {
            right_stick -= 0.1;
        }

        follower.setTeleOpDrive(
                expo(-gamepad.left_stick_y, left_stick),
                expo(-gamepad.left_stick_x, left_stick),
                expo(-gamepad.right_stick_x, right_stick),
                true
        );

        follower.update();
    }

    private void updateTelemetry()
    {
        telemetryM.addData("Left Stick Expo", left_stick);
        telemetryM.addData("Right Stick Expo", right_stick);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        updateDrive(gamepad1);

        updateTelemetry();
    }
}