package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import java.util.List;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Intake {
    private List<VoltageSensor> voltageSensor;

    private final CachingDcMotorEx intake;
    private final CachingDcMotorEx belt;

    // This is still here because we might have to adjust the belt speed
    // Intake belt could easily get ripped off mid-game due to physical factors
    public final double BELT_POWER = 1.0;

    public Intake(HardwareMap hardwareMap)
    {
        voltageSensor = hardwareMap.getAll(VoltageSensor.class);

        belt    = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "Belt"));
        intake  = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "Intake"));

        // Lets just pray this will help our intake
        MotorConfigurationType configBelt = belt.getMotorType().clone();
        configBelt.setAchieveableMaxRPMFraction(1.0);
        belt.setMotorType(configBelt);

        // Just crank up the power pls
        MotorConfigurationType configIntake = intake.getMotorType().clone();
        configIntake.setAchieveableMaxRPMFraction(1.0);
        intake.setMotorType(configIntake);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Our first driver can also activate intake when needed.
    public void update(Gamepad gamepad1, Gamepad gamepad2)
    {
        if(gamepad1.aWasPressed() || gamepad2.aWasPressed())
        {
            start();
        }

        if(gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            stop();
        }

        // Only activate the belt
        // This should useful because intake + belt together take crap ton of voltage
        if(gamepad1.xWasPressed() || gamepad2.xWasPressed())
        {
            setBeltSpeed(BELT_POWER);
        }
    }

    public void start()
    {
        // Find the minimal voltage across all voltage sensors
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensor) {
            double sensorVoltage = sensor.getVoltage();
            if (sensorVoltage > 0) {
                minVoltage = Math.min(minVoltage, sensorVoltage);
            }
        }

        double MAX_VOLTAGE = 14.00;
        double voltageCompensation = minVoltage / MAX_VOLTAGE;

        // Divide intake power based on current voltage
        // This should keep our Shooter more consistent, since the intake won't take full power.
        belt.setPower(BELT_POWER);
        intake.setPower(-1.0 * voltageCompensation);
    }

    public void stop()
    {
        belt.setPower(0);
        intake.setPower(0.0);
    }

    public void setBeltSpeed(double speed)
    {
        belt.setPower(speed);
    }
}
