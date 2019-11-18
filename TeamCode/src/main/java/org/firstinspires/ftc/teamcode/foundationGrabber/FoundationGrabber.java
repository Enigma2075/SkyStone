package org.firstinspires.ftc.teamcode.foundationGrabber;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.arm.Arm;

@Config
public class FoundationGrabber {
    private HardwareMap hardwareMap;

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Servo grabberRight;
    private Servo grabberLeft;

    private Position currentPositionRight = Position.UP;
    private Position currentPositionLeft = Position.UP;

    public enum Position {
        UP(0.0), DOWN(1.0), READY(.25);

        private double numVal;

        Position(double numVal) {
            this.numVal = numVal;
        }

        public double getNumVal() {
            return numVal;
        }
    }

    public FoundationGrabber(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();
        clock = NanoClock.system();

        this.hardwareMap = hardwareMap;

        grabberRight = hardwareMap.servo.get("rightGrabber");
        grabberLeft = hardwareMap.servo.get("leftGrabber");

        grabberLeft.setDirection(Servo.Direction.REVERSE);

        grabberRight.setPosition(Position.UP.getNumVal());
        grabberLeft.setPosition(Position.UP.getNumVal());
    }

    public void waitForMovement(Position position) {
        double start = clock.seconds();

        double timeout = 1;

        switch (position) {
            case UP:
            case DOWN:
                timeout = 1.25;
                break;
        }
        while (!Thread.currentThread().isInterrupted() && clock.seconds() - start < timeout) {
        }
    }

    public void moveToPosition(Position position, Arm.Side side) {
        if(side == Arm.Side.RIGHT) {
            if (currentPositionRight != position) {
                currentPositionRight = position;
                grabberRight.setPosition(position.getNumVal());
            }
        }
        else {
            if(currentPositionLeft != position) {
                currentPositionLeft = position;
                grabberLeft.setPosition(position.getNumVal());
            }
        }
    }

    public void moveToPositionSync(Position position, Arm.Side side) {
        moveToPosition(position, side);
        waitForMovement(position);
    }

    public void stop() {
        moveToPosition(Position.UP, Arm.Side.RIGHT);
        moveToPosition(Position.UP, Arm.Side.LEFT);
    }
}
