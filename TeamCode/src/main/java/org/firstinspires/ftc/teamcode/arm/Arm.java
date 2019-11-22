package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    private HardwareMap hardwareMap;

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Servo right;
    private Servo left;

    private Servo rightRoller;
    private Servo leftRoller;

    public enum Side { RIGHT, LEFT}

    private Position currentPositionRight = Position.UP;
    private Position currentPositionLeft = Position.UP;

    private RollerMode currentRollerModeRight = RollerMode.CLOSE;
    private RollerMode currentRollerModeLeft = RollerMode.CLOSE;

    public enum RollerMode {
        CLOSE(1, 1),
        READY(.5, .5),
        OPEN(.4, .7);

        private double valRight;
        private double valLeft;

        RollerMode(double valRight, double valLeft) {
            this.valRight = valRight;
            this.valLeft = valLeft;
        }

        public double getValRight() {
            return valRight;
        }

        public double getValLeft() {
            return valLeft;
        }
    }

    public enum Position {
        UP(0.0, 0.0, 1.0),
        HOLD(.4, .4, .5),
        READY(.7, .62, 1.0),
        DOWN(1.0, 1.0, .25 ),
        DROP1(.58, .54, 1.0),
        DROP2(.55, .55, 1.0),
        CAP(.6, .6, 1.0);

        private double rightPos;
        private double leftPos;
        private double timeout;

        Position(double leftPos, double rightPos, double timeout) {
            this.leftPos = leftPos;
            this.rightPos = rightPos;
            this.timeout = timeout;
        }

        public double getLeftPos() {
            return leftPos;
        }
        public double getRightPos() {
            return rightPos;
        }
        public double getTimeout() {
            return timeout;
        }
    }

    public Arm(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();
        clock = NanoClock.system();

        this.hardwareMap = hardwareMap;

        right = hardwareMap.servo.get("rightArm");
        left = hardwareMap.servo.get("leftArm");

        rightRoller = hardwareMap.servo.get("rightArmRoller");
        leftRoller = hardwareMap.servo.get("leftArmRoller");

        left.setDirection(Servo.Direction.REVERSE);
        rightRoller.setDirection(Servo.Direction.REVERSE);

        leftRoller.setPosition(RollerMode.CLOSE.getValLeft());
        rightRoller.setPosition(RollerMode.CLOSE.getValRight());

        left.setPosition(Position.UP.getLeftPos());
        right.setPosition(Position.UP.getRightPos());
    }

    public void waitForMovement(Position position) {
        double start = clock.seconds();

        while (!Thread.currentThread().isInterrupted() && clock.seconds() - start < position.getTimeout()) {
        }
    }

    public void setRoller(RollerMode mode, Side side) {
        if(side == Side.RIGHT) {
            if(currentRollerModeRight != mode) {
                currentRollerModeRight = mode;
                rightRoller.setPosition(mode.getValRight());
            }
        }
        else {
            if(currentRollerModeLeft != mode) {
                currentRollerModeLeft = mode;
                leftRoller.setPosition(mode.getValLeft());
            }
        }
    }


    public void moveToPosition(Position position, Side side) {
        if(side == Side.RIGHT) {
            if(currentPositionRight != position) {
                currentPositionRight = position;
                right.setPosition(position.getRightPos());
            }
        }
        else {
            if(currentPositionLeft != position) {
                currentPositionLeft = position;
                left.setPosition(position.getLeftPos());
            }
        }
    }

    public void moveToPositionSync(Position position, Side side) {
        moveToPosition(position, side);
        waitForMovement(position);
    }

    public void stop() {
        setRoller(RollerMode.OPEN, Side.RIGHT);
        setRoller(RollerMode.OPEN, Side.LEFT);
        moveToPosition(Position.HOLD, Side.RIGHT);
        moveToPosition(Position.HOLD, Side.LEFT);
    }
}
