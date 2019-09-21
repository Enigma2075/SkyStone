package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DemoOp", group="Demo")
public class DemoOp extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            double rightPower = -gamepad1.right_stick_y;
            double leftPower = -gamepad1.left_stick_y;

            robot.drivetrain.moveWithPower(rightPower, leftPower);
        }

        robot.stop();
    }
}
