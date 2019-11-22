package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.foundationGrabber.FoundationGrabber;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.sensors.SensorArray;
import org.firstinspires.ftc.teamcode.vision.Vision;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;

    public Vision vision;
    public SensorArray sensorArray;

    public Rev2mDistanceSensor rightDist;
    public Rev2mDistanceSensor leftDist;

    public FoundationGrabber foundationGrabber;

    public AutoSide autoSide;

    public enum AutoSide {
        NONE,
        RED,
        BLUE
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.init(hardwareMap, telemetry, AutoSide.NONE);
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, AutoSide autoSide) {
        init(hardwareMap, telemetry, autoSide);
    }

    private void init(HardwareMap hardwareMap, Telemetry telemetry, AutoSide autoSide) {
        this.autoSide = autoSide;

        sensorArray = new SensorArray(hardwareMap);

        arm = new Arm(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, sensorArray);
        intake = new Intake(hardwareMap, sensorArray);

        vision = new Vision(hardwareMap, autoSide);

        foundationGrabber = new FoundationGrabber(hardwareMap);

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        rightDist = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance");
        leftDist = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance");
    }

    public void moveToDistance(double x, double targetDistance) {

        Rev2mDistanceSensor distSensor = leftDist;
        //double power = .2;
        if(autoSide == AutoSide.BLUE) {
        //    power = -power;
            distSensor = rightDist;
        }

        double dist = distSensor.getDistance(DistanceUnit.INCH) - targetDistance;

        double y = drivetrain.getPoseEstimate().getY();
        double yTarget = y - dist;
        if(autoSide == AutoSide.RED) {
            yTarget = y + dist;
        }

        Trajectory moveToCenterSkyStone = drivetrain.trajectoryBuilder()
                .lineTo(new Vector2d( x,yTarget), new ConstantInterpolator(0))
                .build();
        drivetrain.followTrajectorySync(moveToCenterSkyStone);


        //while(!Thread.currentThread().isInterrupted() && dist > targetDistance) {

            //drivetrain.setDrivePower(new Pose2d(
            //        0,
            //        power,
            //        0
            //));

            //sensorArray.clearRead();
            //drivetrain.update();
            //dist = distSensor.getDistance(DistanceUnit.INCH);


            //telemetry.addData("Dist", dist);
            //telemetry.update();
        //}
        //drivetrain.stop();
    }

    public void grabStone(Arm.Side side) {
        arm.setRoller(Arm.RollerMode.CLOSE, side);
        arm.moveToPositionSync(Arm.Position.DOWN, side);
        arm.moveToPosition(Arm.Position.HOLD, side);
        //arm.setRoller(Arm.RollerMode.STOP, side);
    }

    public void dropStone(Arm.Side side) {
        //arm.moveToPositionSync(Arm.Position.DROP, side);
        //arm.moveToPosition(Arm.Position.HOLD, side);
        arm.setRoller(Arm.RollerMode.OPEN, side);
    }

    public void stop() {
        drivetrain.stop();
    }
}
