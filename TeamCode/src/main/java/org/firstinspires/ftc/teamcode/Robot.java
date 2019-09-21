package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public MechanumDrivetrain drivetrain;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drivetrain = new MechanumDrivetrain(hardwareMap, telemetry);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void stop() {
        drivetrain.stop();
    }
}
