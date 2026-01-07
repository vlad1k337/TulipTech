package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Paths.PathsBlue;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "CommandsBlue")
public class CommandsBlue extends NextFTCOpMode {
    // Pretty self-explanatory, mess around with this values if the robot takes too much time shooting
    // Delay is always in seconds.
    private static final double TIME_TO_SHOOT_PRELOAD = 3;
    private static final double TIME_TO_SHOOT_PPG = 3;
    private static final double TIME_TO_SHOOT_PGP = 3;
    private static final double TIME_TO_SHOOT_GPP = 3;

    private PathsBlue paths;

    private Shooter shooter;
    private Intake intake;
    private SequentialGroup autoCommands;

    // Let NextFTC know about Pedro
    public CommandsBlue()
    {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }

    private SequentialGroup autonomousRoutine()
    {
        PedroComponent.follower().setStartingPose(PathsBlue.startPose);

        paths = new PathsBlue(PedroComponent.follower());

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        InstantCommand prepareShooters = new InstantCommand(() -> {
            shooter.gateClose();
            shooter.setVelocity(Shooter.MID_LINE_VELOCITY);
        });

        InstantCommand startShooter = new InstantCommand(() -> {
            intake.start();
            shooter.gateOpen();
        });

        InstantCommand stopShooter = new InstantCommand(() -> {
            shooter.gateClose();
            shooter.setVelocity(0);
            intake.stop();
        });

        InstantCommand startIntake = new InstantCommand(() -> {
            // Gate is always closed by stopShooter command.
            // This is called just to be sure gate won't be open while intake is running.
            shooter.gateClose();
            intake.start();
        });

        return new SequentialGroup(
                // Score preloads
                prepareShooters,
                new FollowPath(paths.startToShoot).then(
                        // Delay to let shooters reach desired velocity
                        new Delay(1.0)
                ),
                new ParallelGroup(
                        startShooter,
                        new Delay(TIME_TO_SHOOT_PRELOAD)
                ),
                stopShooter,

                // Intake and score PPG
                new FollowPath(paths.moveToPPG).then(
                        startIntake
                ),
                new FollowPath(paths.moveToIntakePPG, true, 0.5).then(
                        prepareShooters
                ),
                new FollowPath((paths.shootPPG)).then(
                        // Delay to let shooters reach desired velocity
                        new Delay(0.75)
                ),
                new ParallelGroup(
                        startShooter,
                        new Delay(TIME_TO_SHOOT_PPG)
                ),
                stopShooter,

                // Intake and score PGP
                new FollowPath(paths.moveToPGP).then(
                        startIntake
                ),
                new FollowPath(paths.moveToIntakePGP, true, 0.5).then(
                        prepareShooters
                ),
                new FollowPath((paths.shootPGP)),
                new ParallelGroup(
                        startShooter,
                        new Delay(TIME_TO_SHOOT_PGP)
                ),
                stopShooter,

                // Intake and score GPP
                new FollowPath(paths.moveToGPP).then(
                        startIntake
                ),
                new FollowPath(paths.moveToIntakeGPP, true, 0.5).then(
                        prepareShooters
                ),
                new FollowPath((paths.shootGPP)),
                new ParallelGroup(
                        startShooter,
                        new Delay(TIME_TO_SHOOT_GPP)
                ),
                stopShooter,

                // Park
                new FollowPath(paths.moveToPGP)
        );
    }

    @Override
    public void onStartButtonPressed()
    {
        autoCommands = autonomousRoutine();
        autoCommands.schedule();
    }

    @Override
    public void onUpdate()
    {
        CommandManager.INSTANCE.run();
        shooter.updateFeedforward();
    }
}
