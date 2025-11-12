package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "TulipLeaveAuto")
public class TulipLeaveAuto extends OpMode {

    private Follower follower;

    private enum pathState {
        STATE_STARTED,
        STATE_CENTER,
        STATE_REST,
    }

    private pathState currentState = pathState.STATE_STARTED;

    private final Pose leftBox    = new Pose(11, 11, 90);

    private final Pose startPoseBlue  = new Pose(64, 6, Math.toRadians(90));
    private final Pose centerPoseBlue = new Pose(56, 37, Math.toRadians(90));

    private final Pose startPoseRed   = new Pose(38.5, 33.5, Math.toRadians(0));
    private final Pose centerPoseRed  = new Pose(72, 72, Math.toRadians(90));

    private Pose startPose, centerPose;

    private PathChain moveToCenter;

    public void buildPath()
    {
        moveToCenter = follower.pathBuilder()
                .addPath(new BezierLine(startPose, centerPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), centerPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate()
    {
        switch(currentState) {
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

    @Override
    public void init()
    {
        startPose  = startPoseRed;
        centerPose = centerPoseRed;

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPath();
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
}
