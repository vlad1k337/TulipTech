package org.firstinspires.ftc.teamcode.subsystems;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightWrapper {
    private Limelight3A camera;

    public LimelightWrapper(HardwareMap hardwareMap)
    {
        camera = hardwareMap.get(Limelight3A.class, "lime");
        camera.pipelineSwitch(0);
        camera.setPollRateHz(30);

        camera.start();
    }

    public void sendTelemetry(TelemetryManager telemetry)
    {
        LLResult result = camera.getLatestResult();

        for(LLResultTypes.FiducialResult fiducial : result.getFiducialResults())
        {
            Pose3D robotPoseFtc = fiducial.getRobotPoseFieldSpace();
            Pose robotPosePedro = new Pose(
                    robotPoseFtc.getPosition().x,
                    robotPoseFtc.getPosition().y,
                    robotPoseFtc.getOrientation().getYaw(),
                    FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            telemetry.addData("Distance", fiducial.getRobotPoseTargetSpace().getPosition().x);
            telemetry.addData("Lime X: ", robotPosePedro.getX());
            telemetry.addData("Lime Y: ", robotPosePedro.getY());
            telemetry.addData("Lime Heading: ", robotPosePedro.getHeading());
            telemetry.addData("AprilTag heading: ", fiducial.getTargetXDegrees());
        }
    }

    public double getHeadingError()
    {
        LLResult result = camera.getLatestResult();

        double error = 0.0;
        for(LLResultTypes.FiducialResult fiducial : result.getFiducialResults())
        {
            error = fiducial.getTargetXDegrees();
        }

        return error;
    }
}
