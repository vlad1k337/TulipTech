package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TulipTele1P extends OpMode {
    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private TelemetryManager telemetryM;

    private final Pose startingPose = new Pose(10, 20);

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        shooter = new Shooter(hardwareMap);
        intake  = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private void updateDrive(Gamepad gamepad)
    {
        follower.update();

        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true
        );
    }

    private void updateTelemetry()
    {
        VoltageSensor controlHubVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.sendTelemetry(telemetryM);

        telemetryM.debug("Voltage: " + controlHubVoltageSensor.getVoltage());

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());

        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        updateDrive(gamepad1);

        shooter.update(gamepad1);

        intake.update(gamepad1);

        updateTelemetry();
    }
}