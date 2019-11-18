package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.SensorArray;

public class Intake {
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private DcMotor pivot;

    private SensorArray sensorArray;
    private  HardwareMap hardwareMap;

    private Mode currentMode = Mode.IDLE;

    private int startingPos;

    private int currentTarget = 0;

    public enum Mode {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    public Intake(HardwareMap hardwareMap, SensorArray sensorArray) {
        this.sensorArray = sensorArray;

        leftMotor = hardwareMap.dcMotor.get("leftIntake");
        rightMotor = hardwareMap.dcMotor.get("rightIntake");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot = hardwareMap.dcMotor.get("pivot");
        startingPos = pivot.getCurrentPosition();
        currentTarget = 0;
        pivot.setTargetPosition(currentTarget);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivot.setPower(.8);
    }

    public void setPivot(int target) {
        if(currentTarget != target) {
            pivot.setTargetPosition(startingPos + target);
            currentTarget = target;
        }
    }


    public void runPivot() {
        pivot.setPower(.2);
    }

    public void stop() {
        pivot.setPower(0);
    }

    public void setMode(Mode mode) {
        if(currentMode != mode) {
            currentMode = mode;

            switch (currentMode) {
                case IDLE:
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                case INTAKE:
                    leftMotor.setPower(1);
                    rightMotor.setPower(1);
                    break;
                case OUTTAKE:
                    leftMotor.setPower(-1);
                    rightMotor.setPower(-1);
                    break;
            }
        }
    }
}
