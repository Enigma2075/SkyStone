package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.foundationGrabber.FoundationGrabber;
import org.firstinspires.ftc.teamcode.intake.Intake;

@TeleOp(name="DriveOp", group="Basic")
public class DriveOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        robot.arm.moveToPosition(Arm.Position.UP, Arm.Side.RIGHT);
        robot.arm.moveToPosition(Arm.Position.UP, Arm.Side.LEFT);

        //robot.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        //double rpm = 0;

        //double startTime = getRuntime();
        //int currentPosition = robot.drivetrain.getEncoder();

        boolean arms = false;
        boolean grabStackPressed = false;
        boolean scorePressed = false;

        robot.intake.setIntakeMode(Intake.IntakeMode.OUTTAKE);
        sleep(200);
        robot.intake.setPivotMode(Intake.PivotMode.INTAKE);
        robot.intake.setIntakeMode(Intake.IntakeMode.IDLE);

        while(opModeIsActive()) {
            robot.sensorArray.clearRead();
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

            if(gamepad1.right_bumper) {
                robot.drivetrain.setDrivePower(new Pose2d(
                        Math.signum(x) * (Math.abs(x)) * .2,
                        Math.signum(y) * (Math.abs(y)) * .2,
                        Math.signum(r) * (Math.abs(r)) * .2
                ));
            }
            else {
                robot.drivetrain.setDrivePower(new Pose2d(
                        Math.signum(x) * (Math.abs(x)) * .7,
                        Math.signum(y) * (Math.abs(y)) * .7,
                        Math.signum(r) * (Math.abs(r)) * .7
                ));
            }

            if(gamepad2.right_trigger > .8) {
                robot.intake.setIntakeMode(Intake.IntakeMode.INTAKE);
            }
            else if(gamepad2.left_trigger > .8) {
                robot.intake.setIntakeMode(Intake.IntakeMode.OUTTAKE);
            }
            else {
                robot.intake.setIntakeMode(Intake.IntakeMode.IDLE);
            }

            if(gamepad1.y) {
                if(!arms) {
                    arms = true;
                    robot.arm.moveToPosition(Arm.Position.UP, Arm.Side.LEFT);
                    robot.arm.moveToPosition(Arm.Position.UP, Arm.Side.RIGHT);
                    robot.arm.setRoller(Arm.RollerMode.CLOSE, Arm.Side.RIGHT);
                    robot.arm.setRoller(Arm.RollerMode.CLOSE, Arm.Side.LEFT);
                }
            }
            else if(gamepad1.a) {
                arms = true;
                robot.arm.moveToPosition(Arm.Position.CAP, Arm.Side.RIGHT);
                robot.arm.moveToPosition(Arm.Position.CAP, Arm.Side.LEFT);
                robot.arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.RIGHT);
                robot.arm.setRoller(Arm.RollerMode.OPEN, Arm.Side.LEFT);
            }
            else {
                if(arms) {
                    arms = false;
                    robot.arm.moveToPosition(Arm.Position.UP, Arm.Side.LEFT);
                    robot.arm.moveToPosition(Arm.Position.UP, Arm.Side.RIGHT);
                    robot.arm.setRoller(Arm.RollerMode.CLOSE, Arm.Side.RIGHT);
                    robot.arm.setRoller(Arm.RollerMode.CLOSE, Arm.Side.LEFT);
                }
            }

            //if(gamepad2.right_bumper) {
            //    robot.intake.setPivotMode(Intake.PivotMode.INTAKE);
            //}
            //else {
            //    robot.intake.setPivotMode(Intake.PivotMode.HOLD);
            //}

            if(gamepad2.y) {
                if(!grabStackPressed) {
                    robot.intake.setMode(Intake.Mode.GRAB_STACK, robot.drivetrain, robot.arm, telemetry);
                    grabStackPressed = true;
                }
            }
            else {
                grabStackPressed = false;
            }

            if(gamepad2.a) {
                if(!scorePressed) {
                    robot.intake.setMode(Intake.Mode.SCORE, robot.drivetrain, robot.arm, telemetry);
                    scorePressed = true;
                }
            }
            else {
                scorePressed = false;
            }

            if(gamepad2.b || gamepad1.b) {
                robot.foundationGrabber.moveToPosition(FoundationGrabber.Position.DOWN, Arm.Side.RIGHT);
                robot.foundationGrabber.moveToPosition(FoundationGrabber.Position.DOWN, Arm.Side.LEFT);
            }
            else {
                robot.foundationGrabber.moveToPosition(FoundationGrabber.Position.UP, Arm.Side.RIGHT);
                robot.foundationGrabber.moveToPosition(FoundationGrabber.Position.UP, Arm.Side.LEFT);
            }

            telemetry.addData("lift:", robot.intake.getLiftPosition());
            telemetry.addData("pivot:", robot.intake.getPivotPosition());

            //telemetry.addData("pivotCo p:", robot.intake.pivotCo.p);
            //telemetry.addData("pivotCo i:", robot.intake.pivotCo.i);
            //telemetry.addData("pivotCo d:", robot.intake.pivotCo.d);
            //telemetry.addData("pivotCo d:", robot.intake.pivotCo.f);
            //telemetry.addData("LiftCo p:", robot.intake.pivotCo.p);
            //telemetry.addData("LiftCo i:", robot.intake.pivotCo.i);
            //telemetry.addData("LiftCo d:", robot.intake.pivotCo.d);
            //telemetry.addData("LiftCo f:", robot.intake.pivotCo.f);

            telemetry.update();
        }
    }
}
