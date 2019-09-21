package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    private HardwareMap hardwareMap;
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive = hardwareMap.dcMotor.get("left_drive");

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void moveWithPower(double rightPower, double leftPower) {
        rightDrive.setPower(rightPower);
        leftDrive.setPower(leftPower);
    }
}
