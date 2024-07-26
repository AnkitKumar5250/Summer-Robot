package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class TankDriveConstants {
    public static final Measure<Distance> TURNING_RADIUS = Meters.of(1);
    public static final Measure<Distance> DISTANCE_PER_ROTATION = Meters.of(1);

    public static final double kPR = 1;
    public static final double kIR = 1;
    public static final double kDR = 1;

    public static final double kPD = 1;
    public static final double kID = 1;
    public static final double kDD = 1;
}
