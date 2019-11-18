package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
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
public class RedAuto extends LinearOpMode {
    Robot robot = null;
    Drivetrain drive = null;
    Arm arm = null;
    Vision vision = null;
    FoundationGrabber grabber = null;

    public enum SkyStonePosition {
        WALL(0), CENTER(8), BRIDGE(17);

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

        arm.moveToPosition(Arm.Position.HOLD, Arm.Side.RIGHT);

        // Move so we are closer to the Stones
        Trajectory moveToSeeSkyStone = drive.trajectoryBuilder()
                .lineTo(new Vector2d( 0,19), new ConstantInterpolator(0))
                .build();

        drive.followTrajectorySync(moveToSeeSkyStone);

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
                        .lineTo(new Vector2d( SkyStonePosition.BRIDGE.getNumVal(),19), new ConstantInterpolator(0))
                        .build();
                drive.followTrajectorySync(moveToRightSkyStone);
            }
        }

        // Disable Tracking when we are done;
        vision.deactivate();

        //Trajectory moveToCenterSkyStone = drive.trajectoryBuilder()
        //        .lineTo(new Vector2d( skyStonePosition.getNumVal() + 2,19), new ConstantInterpolator(0))
        //        .build();
        //drive.followTrajectorySync(moveToCenterSkyStone);

        grabStone(skyStonePosition.getNumVal() + 2, 6);

        // Move to Foundation and drop
        moveToFoundationAndDrop(0);

        if(skyStonePosition == SkyStonePosition.WALL) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-21, 25), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(-21, 5);
        }
        else if(skyStonePosition == SkyStonePosition.CENTER) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-13, 25), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(-13, 5);
        }
        else if(skyStonePosition == SkyStonePosition.BRIDGE) {
            // Move to second SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(-3, 25), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(-3, 5);
        }

        moveToFoundationAndDrop(0);

        if(skyStonePosition == SkyStonePosition.WALL || skyStonePosition == SkyStonePosition.CENTER) {
            // Move to third SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.BRIDGE.getNumVal() + 2, 25), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(SkyStonePosition.BRIDGE.getNumVal() + 2, 5);
        }
        else if(skyStonePosition == SkyStonePosition.BRIDGE) {
            // Move to third SkyStone
            Trajectory moveToSkyStone2 = drive.trajectoryBuilder()
                    .lineTo(new Vector2d(40, 16), new LinearInterpolator(0, 0))
                    .lineTo(new Vector2d(SkyStonePosition.CENTER.getNumVal() + 2, 25), new LinearInterpolator(0, 0))
                    //.back(74)
                    .build();
            drive.followTrajectorySync(moveToSkyStone2);

            grabStone(SkyStonePosition.CENTER.getNumVal() + 2, 5);
        }

            grabber.moveToPosition(FoundationGrabber.Position.READY, Arm.Side.LEFT);

        moveToFoundationAndDrop(10, 2);

        grabber.moveToPosition(FoundationGrabber.Position.DOWN, Arm.Side.LEFT);

        arm.moveToPosition(Arm.Position.UP, Arm.Side.RIGHT);

        Trajectory moveFoundation = drive.trajectoryBuilder()
                .lineTo(new Vector2d(95, 1.5), new LinearInterpolator(0, 0))
                .build();
        drive.followTrajectorySync(moveFoundation);

        grabber.moveToPosition(FoundationGrabber.Position.UP, Arm.Side.LEFT);

        Trajectory park = drive.trajectoryBuilder()
                .lineTo(new Vector2d(65, 1.5), new LinearInterpolator(0, 0))
                //.lineTo(new Vector2d(55, 20), new LinearInterpolator(0, 0))
                .lineTo(new Vector2d(40, 24), new ConstantInterpolator(0))
                .build();
        drive.followTrajectorySync(park);

        //Trajectory park2 = drive.trajectoryBuilder()
        //        .lineTo(new Vector2d(55, 20), new LinearInterpolator(0, 0))
        //        .build();
        //drive.followTrajectorySync(park2);

        //arm.moveToPosition(Arm.Position.HOLD, Arm.Side.RIGHT);

        //Trajectory park1 = drive.trajectoryBuilder()
        //        //.lineTo(new Vector2d(40, 20), new LinearInterpolator(0, 0))
        //        .lineTo(new Vector2d(40, 22), new LinearInterpolator(0, 0))
        //        .build();
        //drive.followTrajectorySync(park1);
    }

    private void grabStone(double x, double distance) {
        // Grab Sky Stone

        arm.moveToPosition(Arm.Position.READY, Arm.Side.LEFT);
        arm.setRoller(Arm.RollerMode.IN, Arm.Side.LEFT);
        robot.moveToDistance(x, distance);
        robot.grabStone(Arm.Side.LEFT);
    }

    private void moveToFoundationAndDrop(double xOffset) {
        moveToFoundationAndDrop(xOffset, 0.0);
    }

    private void moveToFoundationAndDrop(double xOffset, double yOffset) {
        Trajectory moveToFoundation = drive.trajectoryBuilder()
                .lineTo(new Vector2d(40, 20), new LinearInterpolator(0, 0))
                .lineTo(new Vector2d(80.0 + xOffset, 32 + yOffset), new LinearInterpolator(0, 0))
                //.back(74)
                .build();
        drive.followTrajectorySync(moveToFoundation);

        robot.dropStone(Arm.Side.LEFT);
    }

    private boolean followTrajectoryAndFindObject(Trajectory trajectory) {
        boolean found = false;
        drive.followTrajectory(trajectory);
        while(drive.isBusy() && !isStopRequested()) {
            robot.sensorArray.clearRead();
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
