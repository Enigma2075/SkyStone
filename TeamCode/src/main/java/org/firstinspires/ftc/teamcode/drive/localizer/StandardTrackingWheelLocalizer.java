package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

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
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4096;
    public static double WHEEL_RADIUS = 1.96 / 2.0; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7.125 * 2.0; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -4; // in; offset of the lateral wheel

    private ExpansionHubEx hub;

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    private BNO055IMU imu;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(2, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(-2, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftFront");
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightFront");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightRear");

        //imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }


    //@NonNull
    //@Override
    //public double getHeading() {
    //    return imu.getAngularOrientation().firstAngle;
    //}

        @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        wheelPositions.add(encoderTicksToInches(leftEncoder.getCurrentPosition()));
        wheelPositions.add(encoderTicksToInches(rightEncoder.getCurrentPosition()));
        wheelPositions.add(encoderTicksToInches(frontEncoder.getCurrentPosition()));
        return wheelPositions;
    }
}
