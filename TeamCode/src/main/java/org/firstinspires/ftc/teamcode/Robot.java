package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Drivetrain drivetrain;
    public Arm arm;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drivetrain = new Drivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void stop() {
        drivetrain.stop();
    }
}
