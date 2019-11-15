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

    private Servo knockerRight;
    private Servo knockerLeft;

    public enum Side { RIGHT, LEFT}

    private Position currentPositionRight = Position.UP;
    private Position currentPositionLeft = Position.UP;

    private Position currentPositionKnockerRight = Position.HOLD;
    private Position currentPositionKnockerLeft = Position.HOLD;

    public enum Position {
        UP(0.0), HOLD(.4), DOWN(1.0), DROP(.8), CAP(.6);

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

        knockerRight = hardwareMap.servo.get("rightKnocker");
        knockerLeft = hardwareMap.servo.get("leftKnocker");

        left.setDirection(Servo.Direction.REVERSE);

        knockerLeft.setPosition(.5);
        knockerRight.setPosition(.2);

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
                timeout = .75;
                break;
            case DROP:
                timeout = .15;
        }
        while (!Thread.currentThread().isInterrupted() && clock.seconds() - start < timeout) {
        }
    }

    public void moveKnocker(Position position, Side side) {
        if(side == Side.RIGHT) {
            if(currentPositionKnockerRight != position) {
                currentPositionKnockerRight = position;

                if(position == Position.HOLD) {
                    //knockerRight.setPosition(.3);
                    knockerRight.setPosition(.5);
                }
                else if(position == Position.DOWN) {
                    //knockerRight.setPosition(.5);
                    knockerRight.setPosition(1);
                }
                else {
                    knockerRight.setPosition(0);
                    //knockerRight.setPosition(position.getNumVal());
                }
            }
        }
        else {
            if(currentPositionKnockerLeft != position) {
                currentPositionKnockerLeft = position;
                if(position == Position.HOLD) {
                    knockerLeft.setPosition(.5);
                }
                else if(position == Position.DROP) {
                    knockerLeft.setPosition(0);
                }
                else {
                    knockerLeft.setPosition(1);
                }
                //if (position == Position.HOLD) {
                //    knockerLeft.setPosition(0.2);
                //} else if (position == Position.DOWN) {
                //    knockerLeft.setPosition(position.getNumVal());
                //}
                //else {
                //    knockerLeft.setPosition(position.getNumVal());
                //}
            }
        }
    }


    public void moveToPosition(Position position, Side side) {
        if(side == Side.RIGHT) {
            if(currentPositionRight != position) {
                currentPositionRight = position;
                right.setPosition(position.getNumVal());
            }
        }
        else {
            if(currentPositionLeft != position) {
                currentPositionLeft = position;
                if (position == Position.HOLD) {
                    left.setPosition(.43);
                } else {
                    left.setPosition(position.getNumVal());
                }
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
