package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;

import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Paths.PathsRed;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterFeedback;
import org.firstinspires.ftc.teamcode.Subsystem.ShooterFeedforward;

import java.util.List;
import java.util.Optional;

@Autonomous(name = "TulipScoreRed")
public class TulipScoreRed extends OpMode {
    private final boolean DEBUG_INTAKE_ONLY = false;
    private List<LynxModule> allHubs;

    private PathsRed paths;

    private ShooterFeedforward shooter;
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

    final int timeToShoot3 = 4500;
    final int timeToShootPPG = 4500;
    final int timeToShootPGP = 4500;
    final int timeToShootGPP = 4500;

    final int rpmTime3 = 1250;
    final int rpmTimePPG = 1250;
    final int rpmTimePGP = 1250;
    final int rpmTimeGPP = 500;

    @Override
    public void init()
    {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PathsRed.startPose);

        paths = new PathsRed(follower);

        shooter = new ShooterFeedforward(hardwareMap);
        intake = new Intake(hardwareMap);

        shootingTimer = new ElapsedTime();
        RPMTimer = new ElapsedTime();
    }

    @Override
    public void loop()
    {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        shooter.updateFeedforward();

        autonomousPathUpdate();
        follower.update();
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
            intake.stop();
            intake.setBeltSpeed(1.0);
            shooter.gateOpen();
        }

        // Shooting timer ended, continue auto..
        if(shootingTimer.milliseconds() > timeToShoot)
        {
            currentState = nextState;
            nextPath.ifPresent(pathChain -> follower.followPath(pathChain, true));

            shooter.gateClose();
            intake.stop();
            shooter.setVelocity(0.0);
        }
    }

    private void getReadyToShoot()
    {
        shooter.gateClose();
        shooter.setVelocity(ShooterFeedback.midLineVelocity);
        shootingTimer.reset();
        RPMTimer.reset();
    }

    private void autonomousPathUpdate() {
        switch (currentState) {
            case STATE_STARTED:
                currentState = PathState.STATE_SHOOT3;
                follower.followPath(paths.startToShoot, true);
                getReadyToShoot();
                break;

            case STATE_SHOOT3:
                if (!follower.isBusy()) {
                    shootBalls(PathState.STATE_PPG, Optional.of(paths.moveToPPG), timeToShoot3, rpmTime3);
                }
                break;

            case STATE_PPG:
                if(!follower.isBusy()) {
                    intake.start();
                    shooter.gateClose();
                    currentState = PathState.STATE_INTAKE_PPG;
                    follower.followPath(paths.moveToIntakePPG);
                    break;
                }

            case STATE_INTAKE_PPG:
                if(!follower.isBusy()) {
                    currentState = PathState.STATE_SHOOT_PPG;
                    follower.followPath(paths.shootPPG);
                    getReadyToShoot();
                    break;
                }

            case STATE_SHOOT_PPG:
                if(!follower.isBusy()) {
                    shootBalls(PathState.STATE_PGP, Optional.of(paths.moveToPGP), timeToShootPPG, rpmTimePPG);
                    break;
                }

            case STATE_PGP:
                if(!follower.isBusy()) {
                    intake.start();
                    shooter.gateClose();
                    currentState = PathState.STATE_INTAKE_PGP;
                    follower.followPath(paths.moveToIntakePGP);
                    break;
                }

            case STATE_INTAKE_PGP:
                if(!follower.isBusy()) {
                    currentState = PathState.STATE_SHOOT_PGP;
                    follower.followPath(paths.shootPGP);
                    getReadyToShoot();
                    break;
                }

            case STATE_SHOOT_PGP:
                if(!follower.isBusy()) {
                    shootBalls(PathState.STATE_GPP, Optional.of(paths.moveToGPP), timeToShootPGP, rpmTimePGP);
                    break;
                }

            case STATE_GPP:
                if(!follower.isBusy()) {
                    intake.start();
                    shooter.gateClose();
                    currentState = PathState.STATE_INTAKE_GPP;
                    follower.followPath(paths.moveToIntakeGPP);
                    break;
                }

            case STATE_INTAKE_GPP:
                if(!follower.isBusy()) {
                    currentState = PathState.STATE_SHOOT_GPP;
                    follower.followPath(paths.shootGPP);
                    getReadyToShoot();
                    break;
                }

            case STATE_SHOOT_GPP:
                if(!follower.isBusy()) {
                    shootBalls(PathState.STATE_REST, Optional.of(paths.moveToPGP), timeToShootGPP, rpmTimeGPP);
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
