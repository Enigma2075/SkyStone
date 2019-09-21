package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name="DriveOp", group="Basic")
@Disabled
public class DriveOldOp extends LinearOpMode {
    DcMotor rightDrive;
    DcMotor leftDrive;

    CheesyDrive cheesyDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        cheesyDrive = new CheesyDrive();

        rightDrive = hardwareMap.dcMotor.get("right_drive");
        leftDrive = hardwareMap.dcMotor.get("left_drive");

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            double wheel = gamepad1.right_stick_x;
            double throttle = -gamepad1.left_stick_y;

            boolean quickTurn = false;
            if (throttle < .3) {
                quickTurn = true;
            }

            DriveSignal signal = cheesyDrive.cheesyDrive(throttle, wheel, quickTurn);

            rightDrive.setPower(signal.rightMotor);
            leftDrive.setPower(signal.leftMotor);
        }
    }
}
