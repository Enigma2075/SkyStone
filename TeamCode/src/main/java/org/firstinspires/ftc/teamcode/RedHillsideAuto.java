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
public class RedHillsideAuto extends LinearOpMode {
    Robot robot = null;
    Drivetrain drive = null;
    Arm arm = null;
    Vision vision = null;
    FoundationGrabber grabber = null;

    public enum SkyStonePosition {
        WALL(0), CENTER(8), BRIDGE(16);

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
        robot = new Robot(hardwareMap, telemetry, Robot.AutoSide.RED);
        drive = robot.drivetrain;
        arm = robot.arm;
        vision = robot.vision;
        grabber = robot.foundationGrabber;

        SkyStonePosition skyStonePosition = SkyStonePosition.BRIDGE;

        vision.init();
        vision.activate();

        waitForStart();

        if (isStopRequested()) return;

        // Move so we are closer to the Stones
        Trajectory moveToSeeSkyStone = drive.trajectoryBuilder()
                .lineTo(new Vector2d( 0,18.5), new ConstantInterpolator(0))
                .build();

        drive.followTrajectorySync(moveToSeeSkyStone);

        arm.moveToPosition(Arm.Position.HOLD, Arm.Side.RIGHT);
        arm.moveToPosition(Arm.Position.READY, Arm.Side.LEFT);
        arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.LEFT);

        boolean found = findObject();

        if(found) {
            skyStonePosition = SkyStonePosition.WALL;
        }
        else {
            // We didn't find the SkyStone so we move to the center stone
            Trajectory moveToCenterSkyStone = drive.trajectoryBuilder()
                    .lineTo(new Vector2d( SkyStonePosition.CENTER.getNumVal(),19), new ConstantInterpolator(0))
                    .build();
            drive.followTrajectorySync(moveToCenterSkyStone);

            found = findObject();

            if(found) {
                skyStonePosition = SkyStonePosition.CENTER;
            }
            else {
                // We didn't find the SkyStone so we move to the right stone.
                Trajectory moveToRightSkyStone = drive.trajectoryBuilder()
                        .lineTo(new Vector2d( SkyStonePosition.BRIDGE.getNumVal(),27 ), new ConstantInterpolator(0))
                        .build();
                drive.followTrajectorySync(moveToRightSkyStone);
            }
        }

        // Disable Tracking when we are done;
        vision.deactivate();

        grabStone(skyStonePosition.getNumVal() + 1, 0);

        // Move to Foundation and drop
        moveToFoundationAndDrop(20);

        if(skyStonePosition == SkyStonePosition.WALL) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-22, 25), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone2);

            grabStone(-22, 0);
        }
        else if(skyStonePosition == SkyStonePosition.CENTER) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-14, 25), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone2);

            grabStone(-14, 0);
        }
        else if(skyStonePosition == SkyStonePosition.BRIDGE) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-7, 25), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone2);

            grabStone(-7, 0);
        }

        moveToFoundationAndDrop(10);

        if(skyStonePosition == SkyStonePosition.WALL || skyStonePosition == SkyStonePosition.CENTER) {
            // Move to third SkyStone
            Trajectory moveToSkyStone3 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.BRIDGE.getNumVal()+1, 25), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone3);

            grabStone(SkyStonePosition.BRIDGE.getNumVal()+1, 2);
        }
        else if(skyStonePosition == SkyStonePosition.BRIDGE) {
            // Move to third SkyStone
            Trajectory moveToSkyStone3 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.CENTER.getNumVal()+1, 25), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone3);

            grabStone(SkyStonePosition.CENTER.getNumVal()+2, 2);
        }

        grabber.moveToPosition(FoundationGrabber.Position.READY, Arm.Side.LEFT);

        moveToFoundationAndDrop(0, 2);

        grabber.moveToPositionSync(FoundationGrabber.Position.DOWN, Arm.Side.LEFT);

        Trajectory moveFoundation = drive.trajectoryBuilder()
                .lineTo(new Vector2d(60, 35), new LinearInterpolator(Math.toRadians(0), Math.toRadians(-90)))
                .lineTo(new Vector2d(80, 28), new LinearInterpolator(Math.toRadians(-100), 0))
                .build();
        drive.followTrajectorySync(moveFoundation);

        grabber.moveToPosition(FoundationGrabber.Position.UP, Arm.Side.LEFT);

        Trajectory park = drive.trajectoryBuilder()
                .lineTo(new Vector2d(30, 35), new LinearInterpolator(Math.toRadians(-90), 0))
                .build();
        drive.followTrajectorySync(park);
      }

    private void grabStone(double x, double yOffset) {
        // Grab Sky Stone
      Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                .lineTo(new Vector2d(x, 27 + yOffset), new LinearInterpolator(0, 0))
                .build();
        drive.followTrajectorySync(moveToSkyStone2);

        robot.grabStone(Arm.Side.LEFT);
    }

    private void moveToBlock(Trajectory trajectory) {
        drive.followTrajectorySync(trajectory, new ReadyToGrabAtDistance(40));
    }

    private void moveToFoundationAndDrop(double xOffset) {
        moveToFoundationAndDrop(xOffset, 0.0);
    }

    class DropArmAtDistance implements Drivetrain.ExecuteWhileMoving {
        double min;

        public DropArmAtDistance(double min) {
            this.min = min;
        }

        public boolean execute(Pose2d pose) {
            if(pose.getX() > min) {
                robot.arm.moveToPosition(Arm.Position.DROP1, Arm.Side.LEFT);
            }
            //if(pose.getX() > min + 10) {
            //    robot.arm.setRoller(Arm.RollerMode.READY, Arm.Side.LEFT);
            //}
            return true;
        }
    }

    class ReadyToGrabAtDistance implements Drivetrain.ExecuteWhileMoving {
        double max;

        public ReadyToGrabAtDistance(double max) {
            this.max = max;
        }

        public boolean execute(Pose2d pose) {
            if(pose.getX() < max) {
                robot.arm.moveToPosition(Arm.Position.READY, Arm.Side.LEFT);
                robot.arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.LEFT);
            }
            return true;
        }
    }

    private void moveToFoundationAndDrop(double xOffset, double yOffset) {
        Trajectory moveToFoundation = drive.trajectoryBuilder()
                .lineTo(new Vector2d(40, 20), new LinearInterpolator(0, 0))
                .lineTo(new Vector2d(80.5 + xOffset, 32 + yOffset), new LinearInterpolator(0, 0))
                .build();
        drive.followTrajectorySync(moveToFoundation, new DropArmAtDistance(45));

        robot.dropStone(Arm.Side.LEFT);
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
