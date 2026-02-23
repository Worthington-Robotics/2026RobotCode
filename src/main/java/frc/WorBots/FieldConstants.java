package frc.WorBots;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double fieldWidth = Units.inchesToMeters(317.69);
    public static final double fieldLength = Units.inchesToMeters(651.22);
    //Constants needed for targeting
    public static final Translation2d hubPosition = new Translation2d(Units.inchesToMeters(182.11),Units.inchesToMeters(158.84));
    public static final Translation2d[] passExclusionZone = {new Translation2d(3.7980010400002144,3.4057370337953947), new Translation2d(5.524365149091221,5.132589614311369)};
    public static final Translation2d passTarget = new Translation2d(1.9965776218182945,2.478353240555334);
    public static final Translation2d[] allianceZone = {new Translation2d(0.0,-0.03197875149103657), new Translation2d(4.008167105454771,8.5703053995978)};

}
