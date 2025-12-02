package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

@Configurable
@TeleOp
public class TulipTele2P extends OpMode {
    List<LynxModule> allHubs;
    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private TelemetryManager telemetryM;

    // idgaf about starting pose, only heading should matter atp
    // later I will have to create 2 teleops for both alliances, since the heading will differ
    private final Pose startingPose = new Pose(0, 0, Math.toRadians(230));

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

        shooter   = new Shooter(hardwareMap);
        intake    = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private double expo(double input, double exponent)
    {
        return Math.pow(Math.abs(input), exponent) * Math.signum(input);
    }

    private void updateDrive(Gamepad gamepad)
    {
        follower.setTeleOpDrive(
                expo(-gamepad.left_stick_y, 3.0),
                expo(-gamepad.left_stick_x, 3.0),
                expo(-gamepad.right_stick_x, 2.1),
                true
        );

        follower.update();
    }

    private void updateTelemetry()
    {
        shooter.sendTelemetry(telemetryM);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        updateDrive(gamepad1);

        shooter.update(gamepad2);

        intake.update(gamepad2);

        updateTelemetry();
    }
}