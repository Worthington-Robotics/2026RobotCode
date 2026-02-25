package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ShooterIO {
//a
    public static class ShooterIOInputs{
      boolean isConnected = false;
      
      
      public final TalonInputsPositional leader = 
        new TalonInputsPositional("Shooter", "Flywheel Leader");

      public double actualLeaderVelocityRadPerSec = 0.0;
  
      public final TalonInputsPositional hood = 
        new TalonInputsPositional("Shooter", "Hood");

      public double actualHoodPosition = 0.0;
    
    }
    //implement things like make shooter spin at velocity
    /**
     * Updates ShooterIOInputs inputs; also sets the update frequency. 
     * @param input
     */
    public default void updateInputs(ShooterIOInputs input){};


    /**
     * Sets the voltage supplied to the leader.
     * @param volts
     */
    public default void setLeaderVolts(double volts){};

    /**
     * Sets the voltage supplied to the hood.
     * @param volts
     */
    public default void setHoodVolts(double volts){};

    /**
     * Sets the position of the hood using a double value.
     * @param position
     */
    public default void setHoodPosition(double position){};

    /**
     * Sets the position of the hood using a rotation2d.
     * @param rotation
     */
    public default void setHoodPosition(Rotation2d rotation){};

    /**
     * Resets the position of the hood.
     */
    public default void resetHoodPosition(){};

}
