package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MechanumDrivetrain {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    public MechanumDrivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        rightBackDrive = hardwareMap.dcMotor.get("right_back_drive");
        leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");
        leftBackDrive = hardwareMap.dcMotor.get("left_back_drive");

        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to allow moving the robot based on motion in the x, and Y direction. We also allow
     * rotating the orientation of the robot. These are all expected to be within -1 and 1.
     * @param x
     * @param y
     * @param r
     */
    public void move(double x, double y, double r) {
        rightFrontDrive.setPower(Range.clip(y+x-r, -1, 1));
        leftBackDrive.setPower(Range.clip(y+x+r, -1, 1));

        leftFrontDrive.setPower(Range.clip(y-x+r, -1, 1));
        rightBackDrive.setPower(Range.clip(y-x-r, -1, 1));
    }

    public void stop() {
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
    }


    public void moveWithPower(double rightPower, double leftPower) {
//        rightDrive.setPower(rightPower);
//        leftDrive.setPower(leftPower);
    }
}
