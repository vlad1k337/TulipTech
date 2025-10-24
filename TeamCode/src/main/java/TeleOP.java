import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
@TeleOp(name = "TeleMove", group = "TeleOP")
public class TeleOP extends OpMode {
    public static Follower follower;
    private TelemetryManager telemetryM;

    static PoseHistory poseHistory;

    @Override
    public void init() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /** This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry. */
    @Override
    public void init_loop() {
        poseHistory = follower.getPoseHistory();
        telemetryM.debug("I'm gonna kill myself lol");
        telemetryM.debug("I use Linux on the Desktop btw. (so should you!)");
        telemetryM.update();
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the
     * Panels telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);
    }
}
