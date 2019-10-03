package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="DriveOp", group="Basic")
public class DriveOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double r = gamepad1.right_stick_x;

            //double rightFrontInput = Range.clip(y+x-r, -1, 1);
            //double leftRearInput = Range.clip(y+x+r, -1, 1);
            //double leftFrontInput = Range.clip(y-x+r, -1, 1);
            //double rightRearInput = Range.clip(y-x-r, -1, 1);

            telemetry.addData("Heading", "%.1f", robot.drivetrain.getPoseEstimate().getHeading());
            telemetry.addData("External Heading", "%.1f", robot.drivetrain.getExternalHeading());

            //telemetry.addData("DrivePower", "rf,lf,rf,rr = %.1f, %.1f, %.1f, %.1f", rightFrontInput, leftFrontInput, rightRearInput, leftRearInput);

            //telemetry.addData("Joy", "{X, Y, R} = %.1f, %.1f, %.1f", x, y, r);

            robot.drivetrain.move(x, y, r);

            telemetry.update();
        }
    }
}
