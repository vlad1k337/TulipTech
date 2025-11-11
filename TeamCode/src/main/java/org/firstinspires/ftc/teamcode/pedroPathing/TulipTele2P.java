package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
@TeleOp
public class TulipTele2P extends OpMode {
    private DcMotor launchR, launchL;

    private DcMotor belt, intake;
    private Servo gate;

    private Follower follower;
    private TelemetryManager telemetryM;

    public static Pose startingPose;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private String  shootingMode = "NOT SET";
    private double  launchSpeed = 0.0;

    final double backPower   = 0.75;
    final double topPower    = 0.55;
    final double middlePower = 0.50;
    final double boxPower    = 0.40;

    final double beltSpeed = 0.75;


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
        intake  = hardwareMap.get(DcMotor.class, "Intake");

        gate = hardwareMap.get(Servo.class, "Gate");
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        startingPose = new Pose(10, 20);
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
        if(gamepad2.yWasPressed()) {
            launchSpeed  = 0.0;
            shootingMode = "NOT SET";
        }

        if(gamepad2.rightStickButtonWasPressed())
        {
            launchSpeed = 0.35;
            belt.setPower(-beltSpeed);
        }

        if(gamepad2.dpadUpWasPressed())
        {
            launchSpeed  = backPower;
            shootingMode = "BACK";
        } else if(gamepad2.dpadRightWasPressed()) {
            launchSpeed  = topPower;
            shootingMode = "TOP";
        } else if(gamepad2.dpadDownWasPressed()) {
            launchSpeed  = middlePower;
            shootingMode = "MIDDLE";
        } else if(gamepad2.dpadLeftWasPressed()) {
            launchSpeed  = boxPower;
            shootingMode = "BOX";
        }

        if(gamepad2.leftBumperWasPressed()) {
            if (launchSpeed - 0.1 > 0.0) launchSpeed -= 0.1;
        } else if(gamepad2.rightBumperWasPressed()) {
            if(launchSpeed + 0.1 < 1.0) launchSpeed += 0.1;
        }

        if(gamepad2.right_trigger > 0.0)
        {
            gate.setPosition(0.1);
        } else if(gamepad2.left_trigger > 0.0) {
            gate.setPosition(0.0);
        }

        launchR.setPower(-launchSpeed);
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
        if(gamepad2.aWasPressed())
        {
            belt.setPower(beltSpeed);
            intake.setPower(-1.0);
        } else if(gamepad2.bWasPressed()) {
            belt.setPower(0.0);
            intake.setPower(0.0);
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
