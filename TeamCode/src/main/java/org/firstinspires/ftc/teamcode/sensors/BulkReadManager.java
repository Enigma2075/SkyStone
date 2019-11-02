package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

public class BulkReadManager {
    private HardwareMap hardwareMap;
    private ExpansionHubEx hub;
    private boolean hasRead = false;

    RevBulkData lastRead;

    public BulkReadManager(HardwareMap hardwareMap, String hubName) {
        hub = hardwareMap.get(ExpansionHubEx.class, hubName);
    }

    public RevBulkData getBulkData(){
        if(!hasRead) {
            lastRead = hub.getBulkInputData();
            hasRead = true;
        }
        return lastRead;
    }

    public void clearRead() {
        hasRead = false;
    }
}
