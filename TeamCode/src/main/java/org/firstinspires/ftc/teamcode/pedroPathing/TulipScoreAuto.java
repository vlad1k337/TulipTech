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
        STATE_SHOOT3,
        STATE_PPG,
        STATE_INTAKE_PPG,
        STATE_FINISHED_PPG,
        STATE_REST,
    }

    private pathState currentState = pathState.STATE_STARTED;

    private final Pose startPose = new Pose(117, 131, Math.toRadians(217));

    final Pose PPG = new Pose(96, 83.5, Math.toRadians(0));
    final Pose PGP = new Pose(96, 59.5, Math.toRadians(0));
    final Pose GPP = new Pose(96, 35.5, Math.toRadians(0));

    final Pose IntakePPG = new Pose(124, 83.5, Math.toRadians(0));
    final Pose IntakePGP = new Pose(124, 59.5, Math.toRadians(0));
    final Pose IntakeGPP = new Pose(124, 35.5, Math.toRadians(0));

    final Pose shootingPose = new Pose(101.5, 101, Math.toRadians(226));

    private PathChain startToShoot;

    private PathChain moveToPPG, moveToIntakePPG, finishedPPG;

    @Override
    public void init()
    {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap);

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
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        moveToPPG = follower.pathBuilder()
                .addPath(new BezierLine(startPose, PPG))
                .setLinearHeadingInterpolation(startPose.getHeading(), PPG.getHeading())
                .build();

        moveToIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPG, IntakePPG))
                .setLinearHeadingInterpolation(PPG.getHeading(), IntakePPG.getHeading())
                .build();

        finishedPPG = follower.pathBuilder()
                .addPath(new BezierLine(IntakePPG, PPG))
                .setLinearHeadingInterpolation(IntakePPG.getHeading(), PPG.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case STATE_STARTED:
                follower.followPath(moveToPPG, true);
                currentState = pathState.STATE_PPG;
                break;

            case STATE_PPG:
                if (!follower.isBusy()) {
                    intake.start();
                    follower.followPath(moveToIntakePPG, true);
                    currentState = pathState.STATE_INTAKE_PPG;
                }
                break;

            case STATE_INTAKE_PPG:
                if(!follower.isBusy()) {
                    follower.followPath(finishedPPG, true);
                    currentState = pathState.STATE_FINISHED_PPG;
                }

            case STATE_FINISHED_PPG:
                if(!follower.isBusy()) {
                    intake.stop();
                    currentState = pathState.STATE_REST;
                }

            case STATE_REST:
                break;
        }
    }
}
