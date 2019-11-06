package org.firstinspires.ftc.teamcode.foundationGrabber;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class FoundationGrabber {
    private HardwareMap hardwareMap;

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Servo grabber;

    private Position currenPosition = Position.UP;

    public enum Position {
        UP(0.0), DOWN(1.0);

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

        grabber = hardwareMap.servo.get("grabber");

        grabber.setDirection(Servo.Direction.REVERSE);

        grabber.setPosition(Position.UP.getNumVal());
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

    public void moveToPosition(Position position) {
        if(currenPosition != position) {
            currenPosition = position;
            grabber.setPosition(position.getNumVal());
        }
    }

    public void moveToPositionSync(Position position) {
        moveToPosition(position);
        waitForMovement(position);
    }

    public void stop() {
        moveToPosition(Position.UP);
        moveToPosition(Position.UP);
    }
}
