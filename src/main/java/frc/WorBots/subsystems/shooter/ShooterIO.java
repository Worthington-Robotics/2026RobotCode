package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ShooterIO {

    public static class ShooterIOInputs{
      boolean isConnected = false;
      double leaderCurrent;
      double followerCurrent;
      double hoodCurrent;

      public final TalonInputsPositional leader = 
        new TalonInputsPositional("Shooter", "Flywheel Leader");
      public double actualLeaderVelocityRadPerSec = 0;
      public double desiredLeaderVelocityRadPerSec = 0;

    /* 
      public final TalonInputsPositional follower =
        new TalonInputsPositional("Shooter", "Flywheel Follower");
      public double actualFollowerVelocityRadPerSec = 0;
      public double desiredFollowerVelocityRadPerSec = 0;
      */
      public final TalonInputsPositional hood = 
        new TalonInputsPositional("Shooter", "Hood");
      public double actualHoodPosition = 0;
      public double desiredHoodPosition = 0;
    }
    //implement things like make shooter spin at velocity
    public default void updateInputs(ShooterIOInputs input){};


    public default void setLeaderVolts(double volts){};

    public default void setHoodVolts(double volts){};

    public default void setHoodPosition(double position){};

    public default void setHoodPosition(Rotation2d rotation){};

    public default void resetHoodPosition(){};

}
