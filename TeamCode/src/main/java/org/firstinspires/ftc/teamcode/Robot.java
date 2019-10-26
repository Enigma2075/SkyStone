package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Drivetrain drivetrain;
    public Arm arm;
    public Vision vision;

    public Rev2mDistanceSensor distanceSensor;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        arm = new Arm(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        vision = new Vision(hardwareMap);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance");
    }

    public void moveToDistance(Trajectory trajectory, double targetDistance) {
        drivetrain.followTrajectory(trajectory);
        while(drivetrain.isBusy() && distanceSensor.getDistance(DistanceUnit.INCH) > targetDistance) {
            drivetrain.update();
        }
        drivetrain.stop();
    }

    public void grabStone() {
        arm.moveToPositionSync(Arm.Position.DOWN);
        arm.moveToPositionSync(Arm.Position.UP);
    }

    public void dropStone() {
        arm.moveToPositionSync(Arm.Position.DROP);
        arm.moveToPosition(Arm.Position.UP);
    }

    public void stop() {
        drivetrain.stop();
    }
}
