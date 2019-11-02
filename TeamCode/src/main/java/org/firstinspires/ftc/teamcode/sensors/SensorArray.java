package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

public class SensorArray {
    private BulkReadManager rightHub;
    private BulkReadManager leftHub;

    public enum HubSide {
        RIGHT,
        LEFT
    }

    public SensorArray(HardwareMap hardwareMap) {
        rightHub = new BulkReadManager(hardwareMap, "rightHub");
        leftHub = new BulkReadManager(hardwareMap, "leftHub");
    }

    public RevBulkData getBulkData(HubSide side) {
        if(side == HubSide.RIGHT) {
            return rightHub.getBulkData();
        }
        else {
            return leftHub.getBulkData();
        }
    }

    public void clearRead() {
        leftHub.clearRead();
        rightHub.clearRead();
    }
}
