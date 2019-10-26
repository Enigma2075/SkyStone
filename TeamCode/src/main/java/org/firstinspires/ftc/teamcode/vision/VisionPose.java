package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class VisionPose {
    public double x;
    public double y;
    public double z;
    public double roll;
    public double pitch;
    public double heading;

    public VisionPose(double x, double y, double z, double roll, double pitch, double heading) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.heading = heading;
    }
}
