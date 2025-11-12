package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.Shooter;

@Autonomous(name = "TulipScoreAuto")
public class TulipScoreAuto extends OpMode {
    private Shooter shooter;
    private Intake intake;

    private Follower follower;

    private enum pathState {
        STATE_STARTED,
        STATE_CENTER,
        STATE_REST,
    }

    private pathState currentState = pathState.STATE_STARTED;

    private final Pose startPose = new Pose(108, 132, Math.toRadians(270));
    private       Pose centerPose = new Pose(72, 72, Math.toRadians(90));

    final Pose PPG = new Pose(96, 83.5, Math.toRadians(0));
    final Pose PGP = new Pose(96, 59.5, Math.toRadians(0));
    final Pose GPP = new Pose(96, 35.5, Math.toRadians(0));

    private PathChain moveToCenter;

    @Override
    public void init()
    {
        centerPose = PPG;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void loop()
    {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void buildPaths()
    {
        moveToCenter = follower.pathBuilder()
                .addPath(new BezierLine(startPose, centerPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), centerPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case STATE_STARTED:
                follower.followPath(moveToCenter, true);
                currentState = pathState.STATE_CENTER;
                break;

            case STATE_CENTER:
                if (!follower.isBusy()) {
                    currentState = pathState.STATE_REST;
                }
                break;

            case STATE_REST:
                break;
        }
    }
}
