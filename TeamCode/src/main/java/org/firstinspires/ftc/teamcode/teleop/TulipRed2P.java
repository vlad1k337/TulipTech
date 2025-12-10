package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

import java.util.List;

@TeleOp(name = "TulipRed2P")
public class TulipRed2P extends OpMode {
    List<LynxModule> allHubs;
    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private TelemetryManager telemetryM;
    private final Pose startingPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private void updateDrive(Gamepad gamepad) {
        // slows down right stick, makes turning slower
        double TURNING_MULTIPLIER = 0.5;
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x * TURNING_MULTIPLIER,
                false
        );


        follower.update();
    }

    private void updateTelemetry() {
        shooter.updateTelemetry(telemetryM);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        // pls make our loop times smaller
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        updateDrive(gamepad1);

        shooter.update(gamepad2);

        intake.update(gamepad1, gamepad2);

        updateTelemetry();
    }
}