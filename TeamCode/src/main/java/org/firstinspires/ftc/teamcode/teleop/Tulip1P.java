package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

// Test OpMode for one driver to have all the controls
@TeleOp(name = "Tulip1P")
public class Tulip1P extends OpMode {
    private Shooter shooter;
    private Intake intake;

    private Follower follower;
    private TelemetryManager telemetryM;

    private PIDFController headingController;
    private final Pose startingPose = new Pose(72, 72, 0);
    private double targetHeading;
    private boolean headingLock = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        shooter   = new Shooter(hardwareMap);
        intake    = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private double expo(double input)
    {
        return Math.signum(input) * Math.pow(Math.abs(input), 1.4);
    }

    private double getHeadingError()
    {
        double headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), targetHeading)
                * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), targetHeading);

        return headingError;
    }

    private void updateDrive(Gamepad gamepad)
    {
        headingController.setCoefficients(follower.constants.coefficientsHeadingPIDF);
        headingController.updateError(getHeadingError());

        follower.setTeleOpDrive(
                expo(-gamepad.left_stick_y),
                expo(-gamepad.left_stick_x),
                -gamepad.right_stick_x,
                true);

        follower.update();
    }

    // Utility distance calculations, written for applying linear regression on shooting velocity.
    private double calculateDistance(double x1, double y1, double x2, double y2) {
        double xDifference = x2 - x1;
        double yDifference = y2 - y1;
        double distanceSquared = Math.pow(xDifference, 2) + Math.pow(yDifference, 2);
        return Math.sqrt(distanceSquared);
    }

    private void updateTelemetry()
    {
        shooter.updateTelemetry(telemetryM);
        telemetryM.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetryM.addData("X", follower.getPose().getX());
        telemetryM.addData("Y", follower.getPose().getY());
        telemetryM.addData("Distance to (144, 144)", calculateDistance(144, 144, follower.getPose().getX(), follower.getPose().getY()));
        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        updateDrive(gamepad1);

        shooter.update(gamepad1);

        intake.update(gamepad1, gamepad2);

        updateTelemetry();
    }
}