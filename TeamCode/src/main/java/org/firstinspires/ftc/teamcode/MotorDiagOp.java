package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="DC motor diag", group="Diag")
@Disabled
public class MotorDiagOp extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("enc", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
