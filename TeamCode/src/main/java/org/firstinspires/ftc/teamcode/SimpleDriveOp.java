package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.foundationGrabber.FoundationGrabber;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.sensors.SensorArray;

@TeleOp(group="Basic")
public class SimpleDriveOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //SensorArray sensorArray = new SensorArray(hardwareMap);
        Drivetrain drive = new Drivetrain(hardwareMap, null, true);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {

            double y = -gamepad1.left_stick_x;
            double x = -gamepad1.left_stick_y;
            double r = -gamepad1.right_stick_x;

            drive.setDrivePower(new Pose2d(
                    Math.signum(x) * (Math.abs(x) * Math.abs(x) * Math.abs(x)) * .6,
                    Math.signum(y) * (Math.abs(y) * Math.abs(y) * Math.abs(y)) * .6,
                    Math.signum(r) * (Math.abs(r) * Math.abs(r) * Math.abs(r)) * .6
            ));

            telemetry.update();
        }
    }
}
