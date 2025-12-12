    package org.firstinspires.ftc.teamcode.Autonomous;

    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import org.firstinspires.ftc.teamcode.Paths.PathsRed;
    import org.firstinspires.ftc.teamcode.Pedro.Constants;
    import org.firstinspires.ftc.teamcode.Subsystem.Intake;
    import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

    import java.time.Instant;

    import dev.nextftc.core.commands.Command;
    import dev.nextftc.core.commands.CommandManager;
    import dev.nextftc.core.commands.delays.Delay;
    import dev.nextftc.core.commands.groups.ParallelGroup;
    import dev.nextftc.core.commands.groups.SequentialGroup;
    import dev.nextftc.core.commands.utility.InstantCommand;
    import dev.nextftc.extensions.pedro.FollowPath;
    import dev.nextftc.extensions.pedro.PedroComponent;
    import dev.nextftc.ftc.NextFTCOpMode;

    @Autonomous
    public class CommandsRed extends NextFTCOpMode {
        private PathsRed paths;

        private Shooter shooter;
        private Intake intake;

        private SequentialGroup autoCommands;

        public CommandsRed()
        {
            addComponents(
                    new PedroComponent(Constants::createFollower)
            );
        }

        private SequentialGroup autonomousRoutine()
        {
            PedroComponent.follower().setStartingPose(PathsRed.startPose);

            paths = new PathsRed(PedroComponent.follower());

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
                // Just to be sure lmao
                shooter.gateClose();
                intake.start();
            });

            return new SequentialGroup(
                    prepareShooters,
                    new FollowPath(paths.startToShoot).then(
                            new Delay(0.5)
                    ),
                    new ParallelGroup(
                            startShooter,
                            new Delay(4)
                    ),
                    stopShooter,

                    new FollowPath(paths.moveToPPG).then(
                            startIntake
                    ),
                    new FollowPath(paths.moveToIntakePPG).then(
                            prepareShooters
                    ),
                    new FollowPath((paths.shootPPG)),
                    new Delay(0.75),
                    new ParallelGroup(
                            startShooter,
                            new Delay(3)
                    ),
                    stopShooter,

                    new FollowPath(paths.moveToPGP).then(
                            startIntake
                    ),
                    new FollowPath(paths.moveToIntakePGP).then(
                            prepareShooters
                    ),
                    new FollowPath((paths.shootPGP)),
                    new ParallelGroup(
                            startShooter,
                            new Delay(2.5)
                    ),
                    stopShooter,

                    new FollowPath(paths.moveToGPP).then(
                            startIntake
                    ),
                    new FollowPath(paths.moveToIntakeGPP).then(
                            prepareShooters
                    ),
                    new FollowPath((paths.shootGPP)),
                    new ParallelGroup(
                            startShooter,
                            new Delay(2.5)
                    ),
                    stopShooter,

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
