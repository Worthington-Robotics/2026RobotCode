package frc.WorBots.subsystems.shooter;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ShooterIO {

    public static class ShooterIOInputs{
        //nothing in here just yet
        boolean isConnected = false;

        public final TalonInputsPositional fly = 
            new TalonInputsPositional("Shooter", "Flywheel");
        public double velocityRPMfly = 0;
    }
    //implement things like make shooter spin at velocity
    public void updateInputs(ShooterIOInputs input);

    public void setVolts(double volts);

}
