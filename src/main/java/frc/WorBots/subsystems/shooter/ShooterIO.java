package frc.WorBots.subsystems.shooter;

import frc.WorBots.util.HardwareUtils.TalonInputs;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ShooterIO {
    public static class ShooterIOInputs{
      public final TalonInputs leader = 
        new TalonInputs("Shooter", "Flywheel Leader");

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
}
