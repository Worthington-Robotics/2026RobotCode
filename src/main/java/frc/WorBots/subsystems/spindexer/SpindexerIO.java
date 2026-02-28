package frc.WorBots.subsystems.spindexer;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface SpindexerIO {
    public static class SpindexerIOInputs{
        TalonInputsPositional talon = new TalonInputsPositional("Spindexer", "Signals");
        public double spinVelocity = 0;
        public boolean active = false;
        public boolean jammed = false;
    }

    public default void updateInputs(SpindexerIOInputs inputs){}

    public default void setVoltage(double volts){}

    public default void stop(){}
}
