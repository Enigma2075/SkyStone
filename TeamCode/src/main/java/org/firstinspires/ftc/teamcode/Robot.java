package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Drivetrain drivetrain;
    public Arm arm;
    public Vision vision;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        arm = new Arm(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        vision = new Vision(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void stop() {
        drivetrain.stop();
    }
}
