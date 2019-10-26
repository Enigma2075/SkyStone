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

    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, telemetry);
        Drivetrain drive = robot.drivetrain;
        Arm arm = robot.arm;
        Vision vision = robot.vision;

        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeRight(19)
                .build();

        vision.init();
        vision.activate();

        boolean found = false;

        while (!isStopRequested() && !isStarted()) {
            found = vision.findObject();
        }

        //waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);

        for(int i = 0; i < 200 && !found; i++) {
            sleep(2);
            found = vision.findObject();
        }

        if(!found) {
            Trajectory trajectory5 = drive.trajectoryBuilder()
                    .back(8)
                    .build();
            drive.followTrajectorySync(trajectory5);
        }

        for(int i = 0; i < 200 && !found; i++) {
            sleep(2);
            found = vision.findObject();
        }

        if(!found) {
            Trajectory trajectory5 = drive.trajectoryBuilder()
                    .back(8)
                    .build();
            drive.followTrajectorySync(trajectory5);
        }

        // Disable Tracking when we are done;
        vision.deactivate();

        Trajectory trajectory5 = drive.trajectoryBuilder()
                .strafeRight(14)
                .build();

        drive.followTrajectorySync(trajectory5);

        robot.arm.moveToPositionSync(Arm.Position.DOWN);
        robot.arm.moveToPositionSync(Arm.Position.UP);

        Trajectory trajectory1 = drive.trajectoryBuilder()
                //.lineTo(new Vector2d())
                .back(74)
                .build();
        drive.followTrajectorySync(trajectory1);

        /*

        Trajectory trajectory2 = drive.trajectoryBuilder()
                .back(74)
                .build();

        drive.followTrajectorySync(trajectory2);
        Trajectory trajectory3 = drive.trajectoryBuilder()
                .forward(74)
                .build();

        drive.followTrajectorySync(trajectory3);
        Trajectory trajectory4 = drive.trajectoryBuilder()
                .strafeRight(38)
                .build();
        drive.followTrajectorySync(trajectory4);
*/
    }

}
