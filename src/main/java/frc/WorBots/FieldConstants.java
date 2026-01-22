package frc.WorBots;

import static edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.util.Units;
import java.io.IOException;


public class FieldConstants {
    public static final AprilTagFieldLayout aprilTags;



    public static final double fieldLength = Units.inchesToMeters(317.69);
    public static final double fieldWidth = Units.inchesToMeters(651.22);
    
    //AprilTags
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);

    static {
      try {
        aprilTags = AprilTagFieldLayout.loadFromResource(k2026RebuiltWelded.m_resourceFile);
      } catch (IOException e) {
        throw new RuntimeException(e);
        }
    }




    
}
