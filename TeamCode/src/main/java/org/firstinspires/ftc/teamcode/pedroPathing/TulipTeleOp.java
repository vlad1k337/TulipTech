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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    private DcMotor belt;
    private Servo gate;
    private CRServo leftIn, rightIn;

    private Follower follower;
    private TelemetryManager telemetryM;

    public static Pose startingPose;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private String shootingMode = "NOT SET";
    private double launchSpeed = 0.0;

    final double middlePower = 0.50;
    final double backPower   = 0.75;
    final double topPower    = 0.55;
    final double boxPower    = 0.30;

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void initShooters() {
        launchR = hardwareMap.get(DcMotor.class, "launchR");
        launchL = hardwareMap.get(DcMotor.class, "launchL");
    }

    public void initIntake()
    {
        belt    = hardwareMap.get(DcMotor.class, "Belt");
        leftIn  = hardwareMap.get(CRServo.class, "leftIn");
        rightIn = hardwareMap.get(CRServo.class, "rightIn");

        gate = hardwareMap.get(Servo.class, "Gate");
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        startingPose = new Pose(0, 0);
        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        initShooters();

        initIntake();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    private void updateGamepad()
    {
        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
    }

    private void updateShooters()
    {
        if(gamepad1.xWasPressed())
        {
            launchSpeed  = 0.5;
        } else if(gamepad1.yWasPressed()) {
            launchSpeed  = 0.0;
            shootingMode = "NOT SET";
        }

        if(gamepad1.dpadUpWasPressed())
        {
            launchSpeed  = middlePower;
            shootingMode = "MIDDLE";
        } else if(gamepad1.dpadDownWasPressed()) {
            launchSpeed  = backPower;
            shootingMode = "BACK";
        } else if(gamepad1.dpadRightWasPressed()) {
            launchSpeed  = topPower;
            shootingMode = "TOP";
        } else if(gamepad1.dpadLeftWasPressed()) {
            launchSpeed  = boxPower;
            shootingMode = "BOX";
        }

        if(gamepad1.leftBumperWasPressed()) {
            if (launchSpeed - 0.1 > 0.0) launchSpeed -= 0.1;
        } else if(gamepad1.rightBumperWasPressed()) {
            if(launchSpeed + 0.1 < 1.0) launchSpeed += 0.1;
        }

        if(gamepad1.right_trigger > 0.0)
        {
            gate.setPosition(0.1);
        } else if(gamepad1.left_trigger > 0.0) {
            gate.setPosition(0.0);
        }

        launchR.setPower(launchSpeed);
        launchL.setPower(-launchSpeed);

        telemetryM.debug("Shooter Power: " + launchSpeed);
        telemetryM.debug("Shooter Mode:  " + shootingMode);
    }

    private void updateCamera()
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

    private void updateIntake()
    {
        if(gamepad1.aWasPressed())
        {
            belt.setPower(0.6);

            rightIn.setPower(-1.0);
            leftIn.setPower(1.0);
        } else if(gamepad1.bWasPressed()) {
            belt.setPower(0.0);

            rightIn.setPower(0.0);
            leftIn.setPower(0.0);
        }
    }

    private void updateTelemetry()
    {
        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());

        telemetryM.update(telemetry);
    }

    @Override
    public void loop()
    {
        updateGamepad();

        updateShooters();

        updateIntake();

        updateTelemetry();
    }
}
