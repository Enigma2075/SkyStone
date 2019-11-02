package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class Arm {
    private HardwareMap hardwareMap;

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Servo right;
    private Servo left;

    public enum Side { RIGHT, LEFT}

    public enum Position {
        UP(0.0), HOLD(.4), DOWN(1.0), DROP(.8);

        private double numVal;

        Position(double numVal) {
            this.numVal = numVal;
        }

        public double getNumVal() {
            return numVal;
        }
    }

    public Arm(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();
        clock = NanoClock.system();

        this.hardwareMap = hardwareMap;

        right = hardwareMap.servo.get("rightArm");
        left = hardwareMap.servo.get("leftArm");

        left.setDirection(Servo.Direction.REVERSE);

        left.setPosition(Position.UP.getNumVal());
        right.setPosition(Position.UP.getNumVal());
    }

    public void waitForMovement(Position position) {
        double start = clock.seconds();

        double timeout = 1;

        switch (position) {
            case UP:
            case HOLD:
                timeout = .5;
                break;
            case DOWN:
                timeout = 1.25;
                break;
            case DROP:
                timeout = .15;
        }
        while (!Thread.currentThread().isInterrupted() && clock.seconds() - start < timeout) {
        }
    }

    public void moveToPosition(Position position, Side side) {
        if(side == Side.RIGHT) {
            right.setPosition(position.getNumVal());
        }
        else {
            if(position == Position.HOLD) {
                left.setPosition(.5);
            }
            else {
                left.setPosition(position.getNumVal());
            }
        }
    }

    public void moveToPositionSync(Position position, Side side) {
        moveToPosition(position, side);
        waitForMovement(position);
    }

    public void stop() {
        moveToPosition(Position.HOLD, Side.RIGHT);
        moveToPosition(Position.HOLD, Side.LEFT);
    }
}
