package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;
import java.util.Optional;

@Autonomous(name = "TulipScoreRed")
public class TulipScoreRed extends OpMode {
    private final boolean DEBUG_INTAKE_ONLY = false;
    List<LynxModule> allHubs;

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

    final Pose startPose = new Pose(118, 130, Math.toRadians(216));

    final Pose PPG = new Pose(85, 86, Math.toRadians(0));
    final Pose PGP = new Pose(85, 62, Math.toRadians(0));
    final Pose GPP = new Pose(85, 39, Math.toRadians(0));

    final Pose IntakePPG = new Pose(115, 86, Math.toRadians(0));
    final Pose IntakePGP = new Pose(120, 62, Math.toRadians(0));
    final Pose IntakeGPP = new Pose(120, 39, Math.toRadians(0));

    final Pose shootingPose = new Pose(101, 101, Math.toRadians(230));

    private PathChain startToShoot;
    private PathChain moveToPPG, moveToIntakePPG, shootPPG;
    private PathChain moveToPGP, moveToIntakePGP, shootPGP;
    private PathChain moveToGPP, moveToIntakeGPP, shootGPP;

    final int timeToShoot3 = 4000;
    final int timeToShootPPG = 4000;
    final int timeToShootPGP = 5000;
    final int timeToShootGPP = 5000;

    final int rpmTime3 = 0;
    final int rpmTimePPG = 0;
    final int rpmTimePGP = 0;
    final int rpmTimeGPP = 0;

    @Override
    public void init()
    {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

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
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        autonomousPathUpdate();
        follower.update();

        telemetry.addData("path state", currentState);
        telemetry.update();
    }

    private void buildPaths()
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
                .setTangentHeadingInterpolation()
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
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootPGP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakePGP, PGP.mirror(), shootingPose))
                .setLinearHeadingInterpolation(IntakePGP.getHeading(), shootingPose.getHeading())
                .build();

        moveToGPP = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, GPP))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), GPP.getHeading())
                .build();

        moveToIntakeGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP, IntakeGPP))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        shootGPP = follower.pathBuilder()
                .addPath(new BezierCurve(IntakeGPP, GPP.mirror(), shootingPose))
                .setLinearHeadingInterpolation(IntakeGPP.getHeading(), shootingPose.getHeading())
                .build();
    }

    private void shootBalls(PathState nextState, Optional<PathChain> nextPath, int timeToShoot, int rpmTime)
    {
        if(DEBUG_INTAKE_ONLY)
        {
            currentState = nextState;
            nextPath.ifPresent(pathChain -> follower.followPath(pathChain, true));
            return;
        }

        // FUCK THIS IF STATEMENT
        if(RPMTimer.milliseconds() > rpmTime)
        {
            intake.setBeltSpeed(1.0);
            shooter.gateOpen();
        }

        // Shooting timer ended, continue auto..
        if(shootingTimer.milliseconds() > timeToShoot)
        {
            currentState = nextState;
            nextPath.ifPresent(pathChain -> follower.followPath(pathChain, true));

            shooter.gateClose();
            intake.setBeltSpeed(0.0);
            shooter.setVelocity(0.0);
        }
    }

    private void getReadyToShoot()
    {
        shooter.gateClose();
        shooter.setVelocity(Shooter.midLineVelocity);
        shootingTimer.reset();
        RPMTimer.reset();
    }

    private void autonomousPathUpdate() {
        switch (currentState) {
            case STATE_STARTED:
                currentState = PathState.STATE_SHOOT3;
                follower.followPath(startToShoot, true);
                getReadyToShoot();
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
                    getReadyToShoot();
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
                    getReadyToShoot();
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
                    break;
                }

            case STATE_SHOOT_GPP:
                if(!follower.isBusy()) {
                    shootBalls(PathState.STATE_REST, Optional.of(moveToPGP), timeToShootGPP, rpmTimeGPP);
                    break;
                }

            case STATE_REST:
                if(!follower.isBusy()) {
                    intake.stop();
                    shooter.gateClose();
                    shooter.setVelocity(0.0);
                }
                break;
        }
    }
}
