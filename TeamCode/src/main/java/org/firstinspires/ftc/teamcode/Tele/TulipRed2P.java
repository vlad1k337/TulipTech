package org.firstinspires.ftc.teamcode.Tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterFeedforward;

import java.util.List;

@Configurable
@TeleOp(name = "TulipRed2P")
public class TulipRed2P extends OpMode {
    List<LynxModule> allHubs;
    private ShooterFeedforward shooter;
    private Intake intake;

    private Follower follower;
    private TelemetryManager telemetryM;
    private final Pose startingPose = new Pose(0, 0, Math.toRadians(0));

    boolean automatedDrive = false;

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

        shooter = new ShooterFeedforward(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private double expo(double input, double exponent) {
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    private void updateDrive(Gamepad gamepad) {
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x * 0.5,
                    true
            );
        }

        if (gamepad.left_trigger > 0.3 && !follower.isBusy()) {
            follower.followPath(follower.pathBuilder()
                    .addPath(new Path(new BezierPoint(follower.getPose())))
                    .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(180) + Math.atan2(144 - follower.getPose().getY(), 144 - follower.getPose().getX()))
                    .build());
            automatedDrive = true;
        }

        if (automatedDrive && !follower.isBusy()) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

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