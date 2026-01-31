package frc.WorBots.subsystems.intake;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;


public interface IntakeIO {
    public static class IntakeIOInputs{ 
        TalonInputsPositional motor = new TalonInputsPositional("Intake", "Motor");
        boolean isConnected = false;
        double currentDraw = 0.0;



    }

    public default void setIntakeVolts(double volts){}

    public default void updateInputs(IntakeIOInputs inputs){}


}
