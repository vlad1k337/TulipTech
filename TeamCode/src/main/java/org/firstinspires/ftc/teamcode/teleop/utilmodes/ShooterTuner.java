package org.firstinspires.ftc.teamcode.teleop.utilmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.Shooter;

@TeleOp
public class ShooterTuner extends OpMode {
    private TelemetryManager telemetryM;
    private Shooter shooter;

    // 0 for kS, 1 for kV, 2 for kP
    int chosenCoefficient = 0;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.update(gamepad1);
        shooter.updateTelemetry(telemetryM);

        telemetryM.addData("kS", shooter.kSLeft);
        telemetryM.addData("kV", shooter.kV);
        telemetryM.addData("kP", shooter.kP);

        telemetryM.update(telemetry);

        if(gamepad1.dpadRightWasPressed())
        {
            chosenCoefficient += 1;
            chosenCoefficient %= 3;
        }

        if(gamepad1.rightBumperWasPressed())
        {
            switch(chosenCoefficient)
            {
                case 0: shooter.kSLeft += 0.001; break;
                case 1: shooter.kV += 0.001; break;
                case 2: shooter.kP += 0.001; break;
            }
        }

        if(gamepad1.leftBumperWasPressed())
        {
            switch(chosenCoefficient)
            {
                case 0: shooter.kSLeft -= 0.001; break;
                case 1: shooter.kV -= 0.001; break;
                case 2: shooter.kP -= 0.001; break;
            }
        }
    }
}
