package org.firstinspires.ftc.teamcode.Subsystem;
import com.bylazar.telemetry.TelemetryManager;
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
            Pose3D robotPoseFtc = result.getBotpose();
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
