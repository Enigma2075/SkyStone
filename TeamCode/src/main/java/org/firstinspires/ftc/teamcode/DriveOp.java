package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="DriveOp", group="Basic")
public class DriveOp extends LinearOpMode {
    DcMotor rightDrive;
    DcMotor leftDrive;

    CheesyDrive cheesyDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.right_stick_x;
            double y = -gamepad1.right_stick_y;
            double r = -gamepad1.left_stick_y;

            robot.drivetrain.move(x, y, r);
        }
    }
}
