package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

import java.util.List;

@TeleOp(name = "TulipBlue2P")
public class TulipBlue2P extends OpMode {
    List<LynxModule> allHubs;
    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private TelemetryManager telemetryM;
    private final Pose startingPose = new Pose(0, 0, Math.toRadians(180));

    private final double targetHeading = Math.toRadians(312);
    private PIDFController headingController;
    private boolean headingLock = false;

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

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private double getHeadingError()
    {
        if(follower.getCurrentPath() != null)
        {
            return 0;
        }

        double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);

        return headingError;
    }

    private void updateDrive(Gamepad gamepad) {
        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(getHeadingError());

        if(gamepad.rightStickButtonWasPressed())
        {
            headingLock = !headingLock;
        }

        if(headingLock)
        {
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    headingController.run(),
                    false
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    false
            );
        }

        follower.update();
    }

    private void updateTelemetry() {
        shooter.updateTelemetry(telemetryM);

        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        // Please make our loops faster
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        updateDrive(gamepad1);

        shooter.update(gamepad2);

        intake.update(gamepad1, gamepad2);

        updateTelemetry();
    }
}