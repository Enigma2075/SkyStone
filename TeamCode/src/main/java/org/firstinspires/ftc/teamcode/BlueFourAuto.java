package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
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
public class BlueFourAuto extends LinearOpMode {
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
        robot = new Robot(hardwareMap, telemetry, Robot.AutoSide.BLUE);
        drive = robot.drivetrain;
        arm = robot.arm;
        vision = robot.vision;
        grabber = robot.foundationGrabber;

        SkyStonePosition skyStonePosition = SkyStonePosition.BRIDGE;

        arm.setRoller(Arm.RollerMode.FULL_OPEN, Arm.Side.LEFT);
        arm.setRoller(Arm.RollerMode.FULL_OPEN, Arm.Side.RIGHT);

        vision.init();
        vision.activate();

        waitForStart();

        arm.setRoller(Arm.RollerMode.CLOSE, Arm.Side.LEFT);

        if (isStopRequested()) return;

        // Move so we are closer to the Stones
        Trajectory moveToSeeSkyStone = drive.trajectoryBuilder()
                .lineTo(new Vector2d( 0,-18), new ConstantInterpolator(0))
                .build();

        drive.followTrajectorySync(moveToSeeSkyStone);

        arm.moveToPosition(Arm.Position.UP, Arm.Side.LEFT);
        arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.RIGHT);
        arm.moveToPosition(Arm.Position.READY, Arm.Side.RIGHT);

        boolean found = findObject();

        double yOffset = -1.5;
        double xStart = 0;

        if(found) {
            skyStonePosition = SkyStonePosition.WALL;
        }
        else {
            arm.moveToPosition(Arm.Position.HOLD, Arm.Side.RIGHT);
            // We didn't find the SkyStone so we move to the center stone
            Trajectory moveToCenterSkyStone = drive.trajectoryBuilder()
                    .lineTo(new Vector2d( SkyStonePosition.CENTER.getNumVal(),-17), new ConstantInterpolator(0))
                    .build();
            drive.followTrajectorySync(moveToCenterSkyStone);

            found = findObject();
            arm.moveToPosition(Arm.Position.READY, Arm.Side.RIGHT);

            if(found) {
                skyStonePosition = SkyStonePosition.CENTER;
            }
            else {
                // We didn't find the SkyStone so we move to the right stone.
                Trajectory moveToRightSkyStone = drive.trajectoryBuilder()
                        .lineTo(new Vector2d( SkyStonePosition.BRIDGE.getNumVal()+3,-28), new ConstantInterpolator(0))
                        .build();
                drive.followTrajectorySync(moveToRightSkyStone);

                xStart = SkyStonePosition.BRIDGE.getNumVal()+3;
                yOffset = 0;
            }
        }

        // Disable Tracking when we are done;
        vision.deactivate();

        if(skyStonePosition != SkyStonePosition.BRIDGE) {
            Trajectory grabFirst = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(skyStonePosition.getNumVal() + 3, -28 - yOffset), new LinearInterpolator(0, 0))
                    .build();
            drive.followTrajectorySync(grabFirst);
        }

        grabStone(skyStonePosition.getNumVal() + 3, yOffset);

        // Move to Foundation and drop
        moveToFoundationAndDrop(17, 0, xStart);

        if(skyStonePosition == SkyStonePosition.WALL) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, -20), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-22, -28), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone2);

            grabStone(-22, 0);
        }
        else if(skyStonePosition == SkyStonePosition.CENTER) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, -20), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-12, -28), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone2);

            grabStone(-12, 0);
        }
        else if(skyStonePosition == SkyStonePosition.BRIDGE) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, -20), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-6, -28), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone2);

            grabStone(-6, 0);
        }

        moveToFoundationAndDrop(10);

        xStart = 0;
        yOffset = 0;

        if(skyStonePosition == SkyStonePosition.WALL || skyStonePosition == SkyStonePosition.CENTER) {
            // Move to third SkyStone
            Trajectory moveToSkyStone3 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, -20), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.BRIDGE.getNumVal()+2, -28+2), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone3);

            grabStone(SkyStonePosition.BRIDGE.getNumVal()+2, 0);
            xStart = SkyStonePosition.BRIDGE.getNumVal()+2;
        }
        else if(skyStonePosition == SkyStonePosition.BRIDGE) {
            // Move to third SkyStone
            Trajectory moveToSkyStone3 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, -16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.CENTER.getNumVal()+2.5, -28+2.5), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone3);

            grabStone(SkyStonePosition.CENTER.getNumVal()+2.5, 0);
        }

        moveToFoundationAndDrop(2, 0, xStart);

        //NanoClock clock = NanoClock.system();
        //double start = clock.seconds();
        //while (start + .5 > clock.seconds() && !Thread.currentThread().isInterrupted()) {
        //}

        xStart = 0;
        yOffset = 0;

        if(skyStonePosition == SkyStonePosition.WALL) {
            // Move to third SkyStone
            Trajectory moveToSkyStone3 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, -20), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.CENTER.getNumVal()+2, -27), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone3);

            grabStone(SkyStonePosition.CENTER.getNumVal()+2, 0);
            xStart = SkyStonePosition.CENTER.getNumVal()+2;
        }
        else if(skyStonePosition == SkyStonePosition.BRIDGE || skyStonePosition == SkyStonePosition.CENTER) {
            // Move to third SkyStone
            Trajectory moveToSkyStone3 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, -16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.WALL.getNumVal()+2.5, -27), new LinearInterpolator(0, 0))
                    .build();
            moveToBlock(moveToSkyStone3);

            grabStone(SkyStonePosition.WALL.getNumVal()+2.5, 0);
        }

        //grabber.moveToPosition(FoundationGrabber.Position.READY, Arm.Side.RIGHT);

        arm.moveToPosition(Arm.Position.DROP1, Arm.Side.LEFT);
        arm.setRoller(Arm.RollerMode.FULL_OPEN, Arm.Side.LEFT);

        Trajectory moveToFoundation =
                drive.trajectoryBuilder()
                        .lineTo(new Vector2d(40, -21), new LinearInterpolator(0, Math.toRadians(0)))
                        .lineTo(new Vector2d(74, -8), new LinearInterpolator(Math.toRadians(0), Math.toRadians(90)))
                        .build();
        drive.followTrajectorySync(moveToFoundation, new DropArmAtDistance2(40));

        robot.dropStone(Arm.Side.RIGHT);

        //moveToFoundationAndDrop(2, 4, xStart);

        //grabber.moveToPositionSync(FoundationGrabber.Position.DOWN, Arm.Side.RIGHT);

        //Trajectory moveFoundation = drive.trajectoryBuilder()
        //        .lineTo(new Vector2d(70, -28), new LinearInterpolator(Math.toRadians(0), Math.toRadians(90)))
        //        .lineTo(new Vector2d(90, -5), new LinearInterpolator(Math.toRadians(100), 0))
        //        .build();
        //drive.followTrajectorySync(moveFoundation);

        //grabber.moveToPosition(FoundationGrabber.Position.UP, Arm.Side.RIGHT);

        Trajectory park = drive.trajectoryBuilder()
                .lineTo(new Vector2d(60, -14), new LinearInterpolator(Math.toRadians(90), 0))
                .build();
        drive.followTrajectorySync(park);

    }

    private void grabStone(double x, double yOffset) {
        // Grab Sky Stone
        //Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
        //        .lineTo(new Vector2d(x, -28 - yOffset), new LinearInterpolator(0, 0))
        //        .build();
        //drive.followTrajectorySync(moveToSkyStone2);

        robot.grabStone(Arm.Side.RIGHT);
    }

    private void moveToBlock(Trajectory trajectory) {
        drive.followTrajectorySync(trajectory, new ReadyToGrabAtDistance(40));
    }

    private void moveToFoundationAndDrop(double xOffset) {
        moveToFoundationAndDrop(xOffset, 0.0);
    }

    class DropArmAtDistance2 implements Drivetrain.ExecuteWhileMoving {
        double min;

        public DropArmAtDistance2(double min) {
            this.min = min;
        }

        public boolean execute(Pose2d pose) {
            if(pose.getX() > min) {
                robot.arm.moveToPosition(Arm.Position.DROP1, Arm.Side.RIGHT);
            }
            return true;
        }
    }

    class DropArmAtDistance implements Drivetrain.ExecuteWhileMoving {
        double min;

        public DropArmAtDistance(double min) {
            this.min = min;
        }

        public boolean execute(Pose2d pose) {
            if(pose.getX() > min) {
                robot.arm.moveToPosition(Arm.Position.DROP1, Arm.Side.RIGHT);
            }
            else if(pose.getX() > min + 20) {
                robot.arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.RIGHT);
            }
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
                robot.arm.moveToPosition(Arm.Position.READY, Arm.Side.RIGHT);
                robot.arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.RIGHT);
            }
            else if(pose.getX() < max + 40) {
                robot.arm.setRoller(Arm.RollerMode.CLOSE, Arm.Side.RIGHT);
                robot.arm.moveToPosition(Arm.Position.HOLD, Arm.Side.RIGHT);
            }
            else {
                robot.arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.RIGHT);
                robot.arm.moveToPosition(Arm.Position.READY, Arm.Side.RIGHT);
            }
            return true;
        }
    }

    private void moveToFoundationAndDrop(double xOffset, double yOffset) {
        moveToFoundationAndDrop(xOffset, yOffset, 0);
    }

    private void moveToFoundationAndDrop(double xOffset, double yOffset, double xStart) {
        Trajectory moveToFoundation =
                drive.trajectoryBuilder()
                .lineTo(new Vector2d(40, -20), new LinearInterpolator(0, 0))
                .lineTo(new Vector2d(81 + xOffset, -28 - yOffset), new LinearInterpolator(0, 0))
                .build();

        if(xStart != 0) {
            moveToFoundation =
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(xStart, -25), new LinearInterpolator(0, 0))
                            .lineTo(new Vector2d(40, -22.5), new LinearInterpolator(0, 0))
                            .lineTo(new Vector2d(81 + xOffset, -28 - yOffset), new LinearInterpolator(0, 0))
                            .build();
        }

        drive.followTrajectorySync(moveToFoundation, new DropArmAtDistance(45));

        robot.dropStone(Arm.Side.RIGHT);
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
