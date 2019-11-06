package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.foundationGrabber.FoundationGrabber;
import org.firstinspires.ftc.teamcode.intake.Intake;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;

@TeleOp(name="DriveOp", group="Basic")
public class DriveOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        robot.arm.moveToPosition(Arm.Position.HOLD, Arm.Side.RIGHT);
        robot.arm.moveToPosition(Arm.Position.HOLD, Arm.Side.LEFT);

        robot.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        //double rpm = 0;

        //double startTime = getRuntime();
        //int currentPosition = robot.drivetrain.getEncoder();

        boolean arms = false;

        while(opModeIsActive()) {
            //double currentRpm = (((currentPosition - robot.drivetrain.getEncoder()) / TICKS_PER_REV) / (startTime - getRuntime())) * 60.0;

            //if(currentRpm > rpm) {
            //    rpm = currentRpm;
            //}

            //startTime = getRuntime();
            //currentPosition = robot.drivetrain.getEncoder();

            //telemetry.addData("Heading", "%.1f", robot.drivetrain.getPoseEstimate().getHeading());
            //telemetry.addData("External Heading", "%.1f", robot.drivetrain.getExternalHeading());
            //telemetry.addData("RPM", "%.1f", rpm);

            //telemetry.addData("DrivePower", "rf,lf,rf,rr = %.1f, %.1f, %.1f, %.1f", rightFrontInput, leftFrontInput, rightRearInput, leftRearInput);

            //telemetry.addData("Joy", "{X, Y, R} = %.1f, %.1f, %.1f", x, y, r);

            double y = -gamepad1.left_stick_x;
            double x = -gamepad1.left_stick_y;
            double r = -gamepad1.right_stick_x;

            robot.drivetrain.setDrivePower(new Pose2d(
                    Math.signum(x) * (Math.abs(x) * Math.abs(x) * Math.abs(x)) * .8,
                    Math.signum(y) * (Math.abs(y) * Math.abs(y) * Math.abs(y)) * .8,
                    Math.signum(r) * (Math.abs(r) * Math.abs(r) * Math.abs(r)) * .8
            ));

            if(gamepad2.right_trigger > .8 || gamepad1.right_trigger > .8) {
                robot.intake.setMode(Intake.Mode.INTAKE);
            }
            else if(gamepad2.left_trigger > .8 || gamepad1.left_trigger > .8) {
                robot.intake.setMode(Intake.Mode.OUTTAKE);
            }
            else {
                robot.intake.setMode(Intake.Mode.IDLE);
            }

            if(gamepad2.a) {
                if(!arms) {
                    arms = true;
                    robot.arm.moveToPosition(Arm.Position.DOWN, Arm.Side.LEFT);
                    robot.arm.moveToPosition(Arm.Position.DOWN, Arm.Side.RIGHT);
                }
            }
            else if(gamepad2.y) {
                arms = true;
                robot.arm.moveToPosition(Arm.Position.CAP, Arm.Side.RIGHT);
                robot.arm.moveToPosition(Arm.Position.CAP, Arm.Side.LEFT);
                robot.arm.moveKnocker(Arm.Position.DOWN, Arm.Side.RIGHT);
                robot.arm.moveKnocker(Arm.Position.DOWN, Arm.Side.LEFT);
            }
            else {
                if(arms) {
                    arms = false;
                    robot.arm.moveToPosition(Arm.Position.HOLD, Arm.Side.LEFT);
                    robot.arm.moveToPosition(Arm.Position.HOLD, Arm.Side.RIGHT);
                }
            }

            if(gamepad2.right_bumper || gamepad1.right_bumper) {
                robot.intake.setPivot(900);
            }
            else {
                robot.intake.setPivot(0);
            }

            if(gamepad2.b) {
                robot.foundationGrabber.moveToPosition(FoundationGrabber.Position.DOWN);
            }
            else {
                robot.foundationGrabber.moveToPosition(FoundationGrabber.Position.UP);
            }


            telemetry.update();
        }
    }
}
