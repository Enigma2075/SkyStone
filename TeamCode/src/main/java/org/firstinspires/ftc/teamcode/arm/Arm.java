package org.firstinspires.ftc.teamcode.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    private HardwareMap hardwareMap;

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Servo arm;

    public enum Position {
        UP(0.0), DOWN(1.0), DROP(.7);

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

        arm = hardwareMap.servo.get("arm");

        moveToPosition(Position.UP);
    }

    public void waitForMovement(Position position) {
        double start = clock.seconds();

        double timeout = 1;

        switch (position) {
            case UP:
                timeout = .5;
                break;
            case DOWN:
                timeout = 1;
                break;
            case DROP:
                timeout = .16;
        }
        while (!Thread.currentThread().isInterrupted() && clock.seconds() - start < timeout) {
        }
    }

    public void moveToPosition(Position position) {
        arm.setPosition(position.getNumVal());
    }

    public void moveToPositionSync(Position position) {
        moveToPosition(position);
        waitForMovement(position);
    }

    public void stop() {
        moveToPosition(Position.UP);
    }
}
