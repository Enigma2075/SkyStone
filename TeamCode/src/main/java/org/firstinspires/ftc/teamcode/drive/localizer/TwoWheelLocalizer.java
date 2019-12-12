package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.SensorArray;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4096;
    //public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 1.96 / 2.0; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7.0; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 6; // in; offset of the lateral wheel

    private SensorArray sensorArray;

    private DcMotor rightEncoder, frontEncoder;

    private BNO055IMU imu;

    public TwoWheelLocalizer(HardwareMap hardwareMap, SensorArray sensorArray) {
        super(Arrays.asList(
                new Pose2d(2, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        this.sensorArray = sensorArray;

        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightIntake");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftIntake");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    @NonNull
    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

        @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData rightBulkData = sensorArray.getBulkData(SensorArray.HubSide.RIGHT);
        RevBulkData leftBulkData = sensorArray.getBulkData(SensorArray.HubSide.LEFT);

        if (rightBulkData == null || leftBulkData == null) {
            return Arrays.asList(0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(rightBulkData.getMotorCurrentPosition(rightEncoder)));
        wheelPositions.add(-encoderTicksToInches(leftBulkData.getMotorCurrentPosition(frontEncoder)));
        return wheelPositions;
    }
}
