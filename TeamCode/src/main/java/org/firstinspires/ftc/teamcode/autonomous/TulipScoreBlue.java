package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Optional;

@Autonomous(name = "TulipScoreBlue")
public class TulipScoreBlue extends OpMode {
    final boolean DEBUG_INTAKE_ONLY = false;

    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private ElapsedTime shootingTimer, RPMTimer;

    private enum PathState {
        STATE_STARTED,
        STATE_SHOOT3,
        STATE_PPG,
        STATE_INTAKE_PPG,
        STATE_SHOOT_PPG,
        STATE_PGP,
        STATE_INTAKE_PGP,
        STATE_SHOOT_PGP,
        STATE_GPP,
        STATE_INTAKE_GPP,
        STATE_SHOOT_GPP,
        STATE_REST,
    }

    private PathState currentState = PathState.STATE_STARTED;

    final Pose startPose = new Pose(119, 131, Math.toRadians(216)).mirror();

    final Pose PPG = new Pose(85, 84, Math.toRadians(0)).mirror();
    final Pose PGP = new Pose(85, 60, Math.toRadians(0)).mirror();
    final Pose GPP = new Pose(85, 36, Math.toRadians(0)).mirror();

    final Pose IntakePPG = new Pose(118, 84, Math.toRadians(0)).mirror();
    final Pose IntakePGP = new Pose(120, 60, Math.toRadians(0)).mirror();
    final Pose IntakeGPP = new Pose(120, 36, Math.toRadians(0)).mirror();

    final Pose shootingPose = new Pose(101, 101, Math.toRadians(230)).mirror();

    private PathChain startToShoot;
    private PathChain moveToPPG, moveToIntakePPG, shootPPG;
    private PathChain moveToPGP, moveToIntakePGP, shootPGP;
    private PathChain moveToGPP, moveToIntakeGPP, shootGPP;

    final int timeToShoot3 = 5000;
    final int timeToShootPPG = 5000;
    final int timeToShootPGP = 5000;
    final int timeToShootGPP = 5000;

    final int rpmTime3 = 2500;
    final int rpmTimePPG = 2500;
    final int rpmTimePGP = 2500;
    final int rpmTimeGPP = 2500;

    @Override
    public void init()
    {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        shootingTimer = new ElapsedTime();
        RPMTimer = new ElapsedTime();

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
                .addPath(new BezierLine(shootingPose, PPG))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), PPG.getHeading())
                .build();

        moveToIntakePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPG, IntakePPG))
                .setLinearHeadingInterpolation(PPG.getHeading(), IntakePPG.getHeading())
                .setNoDeceleration()
                .build();

        shootPPG = follower.pathBuilder()
                .addPath(new BezierLine(IntakePPG, shootingPose))
                .setLinearHeadingInterpolation(IntakePPG.getHeading(), shootingPose.getHeading())
                .build();

        moveToPGP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, PGP))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), PGP.getHeading())
                .build();

        moveToIntakePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGP, IntakePGP))
                .setLinearHeadingInterpolation(PGP.getHeading(), IntakePGP.getHeading())
                .setNoDeceleration()
                .build();

        shootPGP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakePGP, PGP, shootingPose))
                .setLinearHeadingInterpolation(IntakePGP.getHeading(), shootingPose.getHeading())
                .build();

        moveToGPP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, GPP))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), GPP.getHeading())
                .build();

        moveToIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP, IntakeGPP))
                .setLinearHeadingInterpolation(GPP.getHeading(), IntakeGPP.getHeading())
                .setNoDeceleration()
                .build();

        shootGPP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakeGPP, GPP, shootingPose))
                .setLinearHeadingInterpolation(IntakeGPP.getHeading(), shootingPose.getHeading())
                .build();
    }

    public void shootBalls(PathState nextState, Optional<PathChain> nextPath, int timeToShoot, int rpmTime)
    {
        if(DEBUG_INTAKE_ONLY)
        {
            currentState = nextState;
            nextPath.ifPresent(pathChain -> follower.followPath(pathChain, true));

            return;
        }

        shooter.gateClose();
        shooter.setPower(Shooter.midLinePower);

        if(RPMTimer.milliseconds() > rpmTime)
        {
            intake.start();
            shooter.gateOpen();
        }

        if(shootingTimer.milliseconds() > timeToShoot)
        {
            intake.stop();
            shooter.gateClose();
            shooter.setPower(0.0);

            currentState = nextState;
            nextPath.ifPresent(pathChain -> follower.followPath(pathChain, true));
        }
    }

    public void autonomousPathUpdate() {
        switch (currentState) {
            case STATE_STARTED:
                currentState = PathState.STATE_SHOOT3;
                follower.followPath(startToShoot, true);
                shootingTimer.reset();
                RPMTimer.reset();
                break;

            case STATE_SHOOT3:
                if (!follower.isBusy()) {
                    shootBalls(PathState.STATE_PPG, Optional.of(moveToPPG), timeToShoot3, rpmTime3);
                }
                break;

            case STATE_PPG:
                if(!follower.isBusy()) {
                    intake.start();
                    shooter.gateClose();
                    currentState = PathState.STATE_INTAKE_PPG;
                    follower.followPath(moveToIntakePPG);
                    break;
                }

            case STATE_INTAKE_PPG:
                if(!follower.isBusy()) {
                    currentState = PathState.STATE_SHOOT_PPG;
                    follower.followPath(shootPPG);
                    shootingTimer.reset();
                    RPMTimer.reset();
                    break;
                }

            case STATE_SHOOT_PPG:
                if(!follower.isBusy()) {
                    shootBalls(PathState.STATE_PGP, Optional.of(moveToPGP), timeToShootPPG, rpmTimePPG);
                    break;
                }

            case STATE_PGP:
                if(!follower.isBusy()) {
                    intake.start();
                    shooter.gateClose();
                    currentState = PathState.STATE_INTAKE_PGP;
                    follower.followPath(moveToIntakePGP);
                    break;
                }

            case STATE_INTAKE_PGP:
                if(!follower.isBusy()) {
                    currentState = PathState.STATE_SHOOT_PGP;
                    follower.followPath(shootPGP);
                    shootingTimer.reset();
                    RPMTimer.reset();
                    break;
                }

            case STATE_SHOOT_PGP:
                if(!follower.isBusy()) {
                    shootBalls(PathState.STATE_GPP, Optional.of(moveToGPP), timeToShootPGP, rpmTimePGP);
                    break;
                }

            case STATE_GPP:
                if(!follower.isBusy()) {
                    intake.start();
                    shooter.gateClose();
                    currentState = PathState.STATE_INTAKE_GPP;
                    follower.followPath(moveToIntakeGPP);
                    break;
                }

            case STATE_INTAKE_GPP:
                if(!follower.isBusy()) {
                    currentState = PathState.STATE_SHOOT_GPP;
                    follower.followPath(shootGPP);
                    shootingTimer.reset();
                    RPMTimer.reset();
                    break;
                }

            case STATE_SHOOT_GPP:
                if(!follower.isBusy()) {
                    shootBalls(PathState.STATE_REST, Optional.empty(), timeToShootGPP, rpmTimeGPP);
                    break;
                }

            case STATE_REST:
                if(!follower.isBusy()) {
                    intake.stop();
                    shooter.gateClose();
                    shooter.setPower(0.0);
                }
                break;
        }
    }
}
