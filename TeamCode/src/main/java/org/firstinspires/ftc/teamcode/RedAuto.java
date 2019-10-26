package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class RedAuto extends LinearOpMode {
    Robot robot = null;
    Drivetrain drive = null;
    Arm arm = null;
    Vision vision = null;

    public enum SkyStonePosition {
        LEFT(0), CENTER(-8), RIGHT(-15);

        private double numVal;

        SkyStonePosition(double numVal) {
            this.numVal = numVal;
        }

        public double getNumVal() {
            return numVal;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.drivetrain;
        arm = robot.arm;
        vision = robot.vision;

        SkyStonePosition skyStonePosition = SkyStonePosition.RIGHT;

        vision.init();
        vision.activate();

        waitForStart();

        if (isStopRequested()) return;

        // Move so we are closer to the Stones
        Trajectory moveToSeeSkyStone = drive.trajectoryBuilder()
                .lineTo(new Vector2d( 0,-19), new ConstantInterpolator(0))
                .build();

        drive.followTrajectorySync(moveToSeeSkyStone);

        boolean found = findObject();

        if(found) {
            skyStonePosition = SkyStonePosition.LEFT;
        }
        else {
            // We didn't find the SkyStone so we move to the center stone
            Trajectory moveToCenterSkyStone = drive.trajectoryBuilder()
                    .lineTo(new Vector2d( SkyStonePosition.CENTER.getNumVal(),-19), new ConstantInterpolator(0))
                    .build();
            drive.followTrajectorySync(moveToCenterSkyStone);

            found = findObject();

            if(found) {
                skyStonePosition = SkyStonePosition.CENTER;
            }
            else {
                // We didn't find the SkyStone so we move to the right stone.
                Trajectory moveToRightSkyStone = drive.trajectoryBuilder()
                        .lineTo(new Vector2d( SkyStonePosition.RIGHT.getNumVal(),-19), new ConstantInterpolator(0))
                        .build();
                drive.followTrajectorySync(moveToRightSkyStone);
            }
        }

        // Disable Tracking when we are done;
        vision.deactivate();

        grabStone(skyStonePosition.getNumVal(), 6.5);

        // Move to Foundation and drop
        moveToFoundationAndDrop(0);

        if(skyStonePosition == SkyStonePosition.LEFT) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(-40, -16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(32, -19), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(32, 3);
        }
        else if(skyStonePosition == SkyStonePosition.CENTER) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(-40, -16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(20, -19), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(20, 3);
        }
        else if(skyStonePosition == SkyStonePosition.RIGHT) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(-40, -16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(8, -19), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(8, 3);
        }

        moveToFoundationAndDrop(-10);

        Trajectory moveFoundation = drive.trajectoryBuilder()
                .lineTo(new Vector2d(-100, 5), new LinearInterpolator(0, 0))
                .build();
        drive.followTrajectorySync(moveFoundation);

        Trajectory park = drive.trajectoryBuilder()
                .lineTo(new Vector2d(-50, 0), new LinearInterpolator(0, 0))
                .build();
        drive.followTrajectorySync(park);

        Trajectory park2 = drive.trajectoryBuilder()
                .lineTo(new Vector2d(-50, -25), new LinearInterpolator(0, 0))
                .build();
        drive.followTrajectorySync(park2);

        Trajectory park1 = drive.trajectoryBuilder()
                .lineTo(new Vector2d(-40, -25), new LinearInterpolator(0, 0))
                .build();
        drive.followTrajectorySync(park1);
    }

    private void grabStone(double x, double distance) {
        // Grab Sky Stone
        Trajectory moveToGrabSkyStone2 = drive.trajectoryBuilder()
                .lineTo(new Vector2d(x,-26), new ConstantInterpolator(0))
                .build();

        robot.moveToDistance(moveToGrabSkyStone2, distance);
        robot.grabStone();
    }

    private void moveToFoundationAndDrop(double offset) {
        Trajectory moveToFoundation = drive.trajectoryBuilder()
                .lineTo(new Vector2d(-40, -16), new LinearInterpolator(0, 0))
                .lineTo(new Vector2d(-90.0 + offset, -30), new LinearInterpolator(0, 0))
                //.back(74)
                .build();
        drive.followTrajectorySync(moveToFoundation);

        robot.dropStone();
    }

    private boolean followTrajectoryAndFindObject(Trajectory trajectory) {
        boolean found = false;
        drive.followTrajectory(trajectory);
        while(drive.isBusy() && !isStopRequested()) {
            drive.update();
            if(!found) {
                found = vision.findObject();
            }
        }
        return found;
    }

    private boolean findObject() {
        boolean found = false;
        for (int i = 0; i < 200 && !found; i++) {
            sleep(2);
            found = vision.findObject();
        }
        return found;
    }

}
