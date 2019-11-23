package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.foundationGrabber.FoundationGrabber;
import org.firstinspires.ftc.teamcode.vision.Vision;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class ParkAuto extends LinearOpMode {
    Robot robot = null;
    Drivetrain drive = null;
    Arm arm = null;
    Vision vision = null;
    FoundationGrabber grabber = null;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, Robot.AutoSide.BLUE);
        drive = robot.drivetrain;
        arm = robot.arm;
        vision = robot.vision;
        grabber = robot.foundationGrabber;

        waitForStart();

        if (isStopRequested()) return;

        // Move so we are closer to the Stones
        Trajectory moveToSeeSkyStone = drive.trajectoryBuilder()
                .lineTo(new Vector2d(5, -19), new ConstantInterpolator(0))
                .build();

        drive.followTrajectorySync(moveToSeeSkyStone);

    }
}
