package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
public class TestAutoOp extends LinearOpMode {
    Robot robot = null;
    Drivetrain drive = null;
    Arm arm = null;
    Vision vision = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        drive = robot.drivetrain;
        arm = robot.arm;
        vision = robot.vision;

        Trajectory moveToSeeSkyStone = drive.trajectoryBuilder()
                .strafeRight(19)
                .build();

        vision.init();
        vision.activate();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(moveToSeeSkyStone);

        boolean found = false;
        found = findObject();

        if (!found) {
            Trajectory moveToNextSkyStone = drive.trajectoryBuilder()
                    .back(8)
                    .build();
            found = followTrajectoryAndFindObject(moveToNextSkyStone);
        }

        if(!found) {
            found = findObject();

            if (!found) {
                Trajectory moveToNextSkyStone = drive.trajectoryBuilder()
                        .back(8)
                        .build();
                drive.followTrajectorySync(moveToNextSkyStone);
            }
        }

        // Disable Tracking when we are done;
        vision.deactivate();

        Trajectory moveToGrabSkyStone = drive.trajectoryBuilder()
                .strafeRight(14)
                .build();

        drive.followTrajectorySync(moveToGrabSkyStone);

        robot.grabStone();

        Trajectory trajectory1 = drive.trajectoryBuilder()
                //.lineTo(new Vector2d())
                .back(74)
                .build();
        drive.followTrajectorySync(trajectory1);

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
