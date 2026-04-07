package frc.WorBots;

import static edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

public class FieldConstants {
  public static final AprilTagFieldLayout aprilTags;

  // AprilTags
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2026RebuiltWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final double fieldWidth = Units.inchesToMeters(317.69);
  public static final double fieldLength = Units.inchesToMeters(651.22); 
  // Constants needed for targeting
  public static final Translation2d hubPosition = new Translation2d(Units.inchesToMeters(182.11),
      Units.inchesToMeters(158.84));
  // public static final Translation2d[] passExclusionZone = { new Translation2d(3.7980010400002144, 3.4057370337953947),
  //     new Translation2d(5.524365149091221, 5.132589614311369) };
  public static final double passExclusionMargin = Units.inchesToMeters(1);
  public static final Translation2d[] passExclusionZone = { new Translation2d(Units.inchesToMeters(182.11-22.0) - passExclusionMargin, Units.inchesToMeters(158.84 - 22.0) - passExclusionMargin),
      new Translation2d(Units.inchesToMeters(182.11+25.0) + passExclusionMargin, Units.inchesToMeters(158.84 + 25.0) + passExclusionMargin) };
  public static final Translation2d passTarget = new Translation2d(1.0, 1.0);
  public static final Translation2d[] allianceZone = { new Translation2d(0.0, 0.0),
      new Translation2d(Units.inchesToMeters(182.11-23.5), fieldWidth) };


  // Field Zones 
  public static final double trenchMargin = Units.inchesToMeters(22);
  public static final Translation2d[] blueZone = { new Translation2d(0,0),
    new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(317.69))};

  public static final Translation2d[] neutralZone = { new Translation2d(Units.inchesToMeters(182.11),0),
    new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(317.69))};

  public static final Translation2d[] redZone = { new Translation2d(Units.inchesToMeters(469.11), 0),
    new Translation2d(Units.inchesToMeters(651.22),Units.inchesToMeters(317.69))};

  // Constants for trench     
  public static final Translation2d[] redTopTrench = { new Translation2d(Units.inchesToMeters(160)-trenchMargin,0),
    new Translation2d(Units.inchesToMeters(200)+trenchMargin,Units.inchesToMeters(62.67))};
  
  public static final Translation2d[] redBottomTrench = { new Translation2d(Units.inchesToMeters(160)-trenchMargin,Units.inchesToMeters(255.34)),
    new Translation2d(Units.inchesToMeters(200)+trenchMargin, Units.inchesToMeters(317.69))};

  public static final Translation2d[] blueTopTrench = {new Translation2d(Units.inchesToMeters(447)-trenchMargin,0),
    new Translation2d(Units.inchesToMeters(487.11)+trenchMargin, Units.inchesToMeters(62.67))};

  public static final Translation2d[] blueBottomTrench = { new Translation2d(Units.inchesToMeters(447)-trenchMargin, Units.inchesToMeters(255.34)),
    new Translation2d(Units.inchesToMeters(487)+trenchMargin, Units.inchesToMeters(317.69))};


}
