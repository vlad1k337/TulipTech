package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class TulipTeleOp extends OpMode {
    private Supplier<PathChain> pathChain;
    private DcMotor launchR, launchL;

    private Follower follower;
    private TelemetryManager telemetryM;

    public static Pose startingPose;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private double launchSpeedRight = 0.0;
    private double launchSpeedLeft  = 0.0;

    public void initAprilTag()
    {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void initShooters()
    {
        launchR = hardwareMap.get(DcMotor.class, "launch R");
        launchL = hardwareMap.get(DcMotor.class, "launch L");
    }

    @Override
    public void init()
    {
        follower = Constants.createFollower(hardwareMap);

        startingPose = new Pose(0, 0);
        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        initAprilTag();
        initShooters();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72, 72))))
                .setConstantHeadingInterpolation(45.0)
                .build();
    }

    @Override
    public void start()
    {
        follower.startTeleopDrive();
        follower.update();
    }

    public void updateShooters()
    {
        if(gamepad1.xWasPressed())
        {
            launchSpeedLeft  = 0.5;
            launchSpeedRight = 0.5;
        } else if(gamepad1.yWasPressed()) {
            launchSpeedLeft  = 0.0;
            launchSpeedRight = 0.0;
        }

        if(gamepad1.leftBumperWasPressed()) {
            if (launchSpeedLeft - 0.1 > 0.0) launchSpeedLeft -= 0.1;
            if (launchSpeedRight - 0.1 > 0.0) launchSpeedRight -= 0.1;
        } else if(gamepad1.rightBumperWasPressed()) {
            if(launchSpeedLeft  + 0.1 < 1.0) launchSpeedLeft  += 0.1;
            if(launchSpeedRight + 0.1 < 1.0) launchSpeedRight += 0.1;
        }

        telemetryM.debug("Left Shooter power: " + launchSpeedLeft);
        telemetryM.debug("Right Shooter power: " + launchSpeedRight);

        launchR.setPower(-launchSpeedRight);
        launchL.setPower(launchSpeedLeft);
    }

    public void updateCamera()
    {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(AprilTagDetection detection : currentDetections)
        {
            if(detection.metadata != null)
            {
                telemetryM.debug("Tag Name: " + detection.metadata.name);
                telemetryM.debug("Tag Pose (x, y, z): " + detection.ftcPose.x + ", " + detection.ftcPose.y + ", " + detection.ftcPose.z);
            }
        }
    }

    @Override
    public void loop()
    {
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        updateShooters();

        updateCamera();

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());

        telemetryM.update(telemetry);

    }
}
