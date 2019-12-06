package org.firstinspires.ftc.teamcode.intake;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RedAuto;
import org.firstinspires.ftc.teamcode.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.SensorArray;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Intake {
    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private ExpansionHubMotor pivot;

    private ExpansionHubMotor lift;

    private Servo tilt;

    private DigitalChannel limit;

    private SensorArray sensorArray;
    private  HardwareMap hardwareMap;

    private int startingPivotPos;
    private int startingLiftPos;

    private IntakeMode currentIntakeMode = IntakeMode.IDLE;
    private PivotMode currentPivotMode = PivotMode.HOLD;
    private LiftMode currentLiftMode = LiftMode.DOWN;
    private Mode currentMode = Mode.NONE;

    private double currentTiltPos = 0;

    public enum Mode {
        NONE,
        INTAKE,
        HOLD,
        PLACE,
        GRAB_STACK,
        SCORE
    }

    public enum PivotMode {
        INTAKE(490),
        //MOVE_LIFT(490),
        HOLD(0),
        PLACE(-410);

        private int val;

        PivotMode(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }

    }

    public enum LiftMode {
        DOWN(0),
        GRAB_STACK(0),
        TILT(500),
        SCORE(600),
        UP(1100);

        private int val;

        LiftMode(int val) {
            this.val = val;
        }

        public int getVal() {
            return val;
        }

    }

    public enum IntakeMode {
        IDLE,
        INTAKE,
        OUTTAKE,
        PLACE
    }

    //public PIDFCoefficients liftCo;
    //public PIDFCoefficients pivotCo;

    public Intake(HardwareMap hardwareMap, SensorArray sensorArray) {
        this.sensorArray = sensorArray;

        limit = hardwareMap.digitalChannel.get("intakeLimit");

        leftMotor = hardwareMap.dcMotor.get("leftIntake");
        rightMotor = hardwareMap.dcMotor.get("rightIntake");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot = hardwareMap.get(ExpansionHubMotor.class, "pivot");

        startingPivotPos = pivot.getCurrentPosition();
        currentPivotMode = PivotMode.HOLD;
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setTargetPosition(startingPivotPos);
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivot.setPower(1);

        lift = hardwareMap.get(ExpansionHubMotor.class, "lift");
        startingLiftPos = lift.getCurrentPosition();
        currentLiftMode = LiftMode.DOWN;
        lift.setTargetPosition(startingLiftPos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

        lift.setPositionPIDFCoefficients(10);
        pivot.setPositionPIDFCoefficients(8);

        //liftCo = lift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //pivotCo = pivot.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        tilt = hardwareMap.servo.get("tilt");
        tilt.setPosition(0);
    }

    public void setMode(Mode mode, Drivetrain drive, Telemetry telemetry) {
        switch(currentMode) {
            case NONE:
                switch (mode) {
                    case GRAB_STACK:
                        _setPivotMode(PivotMode.INTAKE);
                        _setLiftMode(LiftMode.TILT);
                        tilt.setPosition(1);
                        while (lift.getCurrentPosition() < startingLiftPos + LiftMode.TILT.getVal() - 75 && !Thread.currentThread().isInterrupted()) {

                        }
                        _setLiftMode(LiftMode.GRAB_STACK);
                        currentMode = mode;
                        break;
                }
                break;
            case GRAB_STACK:
                switch (mode) {
                    case GRAB_STACK:
                        tilt.setPosition(0);
                        _setPivotMode(PivotMode.INTAKE);
                        _setLiftMode(LiftMode.TILT);
                        while (lift.getCurrentPosition() < startingLiftPos + LiftMode.TILT.getVal() - 50 && !Thread.currentThread().isInterrupted()) {

                        }
                        _setLiftMode(LiftMode.DOWN);
                        while (lift.getCurrentPosition() > startingLiftPos + LiftMode.DOWN.getVal() + 50 && !Thread.currentThread().isInterrupted()) {

                        }
                        currentMode = Mode.NONE;
                        break;
                    case SCORE:
                        drive.setDrivePower(new Pose2d(-.2, 0, 0));
                        tilt.setPosition(1);
                        _setLiftMode(LiftMode.UP);
                        while (lift.getCurrentPosition() < startingLiftPos + LiftMode.UP.getVal() - 50 && !Thread.currentThread().isInterrupted()) {
                        }
                        _setPivotMode(PivotMode.PLACE);
                        while (pivot.getCurrentPosition() > startingPivotPos + PivotMode.PLACE.getVal() + 50 && !Thread.currentThread().isInterrupted()) {
                        }
                        _setIntakeMode(IntakeMode.PLACE);
                        NanoClock clock = NanoClock.system();
                        double start = clock.seconds();
                        while (start + 1.0 > clock.seconds() && !Thread.currentThread().isInterrupted()) {

                        }
                        currentMode = mode;
                        break;
                }
                break;
            case SCORE:
                switch (mode) {
                    case SCORE:
                        drive.setDrivePower(new Pose2d(-.2, 0, 0));
                        _setPivotMode(PivotMode.INTAKE);
                        while (pivot.getCurrentPosition() < startingPivotPos + PivotMode.INTAKE.getVal() - 100 && !Thread.currentThread().isInterrupted()) {

                        }

                        _setLiftMode(LiftMode.TILT);
                        while (lift.getCurrentPosition() > startingLiftPos + LiftMode.SCORE.getVal() + 75 && !Thread.currentThread().isInterrupted()) {

                        }

                        NanoClock clock = NanoClock.system();
                        double start = clock.seconds();
                        while (start + .5 > clock.seconds() && !Thread.currentThread().isInterrupted()) {

                        }

                        _setLiftMode(LiftMode.SCORE);
                        while (lift.getCurrentPosition() > startingLiftPos + LiftMode.SCORE.getVal() + 75 && !Thread.currentThread().isInterrupted()) {

                        }

                        //drive.setPoseEstimate(new Pose2d(0,0,0));
                        //Trajectory moveToCenterSkyStone = drive.trajectoryBuilder()
                        //        .forward(9)
                        //        .build();
                        //drive.followTrajectorySync(moveToCenterSkyStone);

                        currentMode = Mode.PLACE;
                        break;
                }
                break;
            case PLACE:
                switch (mode) {
                    case SCORE:
                        _setLiftMode(LiftMode.DOWN);
                        currentMode = Mode.GRAB_STACK;
                        break;
                    case GRAB_STACK:
                        tilt.setPosition(0);
                        _setPivotMode(PivotMode.INTAKE);
                        _setLiftMode(LiftMode.TILT);
                        while (lift.getCurrentPosition() < startingLiftPos + LiftMode.TILT.getVal() - 50 && !Thread.currentThread().isInterrupted()) {

                        }
                        _setLiftMode(LiftMode.DOWN);
                        while (lift.getCurrentPosition() > startingLiftPos + LiftMode.DOWN.getVal() + 50 && !Thread.currentThread().isInterrupted()) {

                        }
                        currentMode = Mode.NONE;
                        break;
                }
        }
    }

    public void _setLiftMode(LiftMode mode) {
        if(mode == currentLiftMode) {
            return;
        }

        lift.setTargetPosition(startingLiftPos + mode.getVal());
        currentLiftMode = mode;
    }

    public void setPivotMode(PivotMode mode) {
        if (mode == currentPivotMode || (currentMode != Mode.NONE
                && (currentMode != Mode.PLACE
                        || currentMode != Mode.GRAB_STACK)
                && mode != PivotMode.INTAKE )
        )
         {
            return;
        }
        _setPivotMode(mode);
    }

    public void _setPivotMode(PivotMode mode) {
        //switch (mode) {
        //    case INTAKE:
        //    case HOLD:
        //        break;
        //    case PLACE:
        //        RevBulkData data = sensorArray.getBulkData(SensorArray.HubSide.RIGHT);
        //        if(data == null || data.getMotorCurrentPosition(lift) - startingLiftPos < LiftMode.UP.getVal() - 100) {
        //            return;
        //        }
        //        break;

        //}

        pivot.setTargetPosition(startingPivotPos + mode.getVal());
        currentPivotMode = mode;
    }

    public void stop() {
        pivot.setPower(0);
    }

    public void setIntakeMode(IntakeMode mode) {
        //if(currentMode != Mode.NONE
        //&& ((currentMode != Mode.PLACE
        //|| currentMode != Mode.GRAB_STACK)
        //&& (mode != IntakeMode.INTAKE && mode != IntakeMode.OUTTAKE ))) {
        //    return;
        //}

        _setIntakeMode(mode);
    }

    private void _setIntakeMode(IntakeMode mode) {
        RevBulkData data = sensorArray.getBulkData(SensorArray.HubSide.RIGHT);

        if(data == null) {
            return;
        }

        boolean limitState = data.getDigitalInputState(limit);
        if(mode == IntakeMode.INTAKE
                && currentPivotMode == PivotMode.PLACE){
            mode = IntakeMode.PLACE;
        }
        else if(mode == IntakeMode.INTAKE
                && !limitState
        && currentPivotMode != PivotMode.PLACE){
            mode = IntakeMode.IDLE;
        }

        if(currentIntakeMode != mode) {
            currentIntakeMode = mode;

            switch (currentIntakeMode) {
                case IDLE:
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    break;
                case INTAKE:
                    leftMotor.setPower(1);
                    rightMotor.setPower(1);
                    break;
                case OUTTAKE:
                    leftMotor.setPower(-1);
                    rightMotor.setPower(-1);
                    break;
                case PLACE:
                    leftMotor.setPower(1.0);
                    rightMotor.setPower(1.0);
                    break;
            }
        }
    }

    public int getPivotPosition() {
        return pivot.getCurrentPosition();
    }

    public int getLiftPosition() {
        return lift.getCurrentPosition();
    }
}
