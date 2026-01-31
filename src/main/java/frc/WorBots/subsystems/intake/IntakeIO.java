package frc.WorBots.subsystems.intake;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;


public interface IntakeIO {
    public static class IntakeIOInputs{ 
        
        TalonInputsPositional intakeMotor = new TalonInputsPositional("Intake", "Intake Motor");
        TalonInputsPositional extendingMotor = new TalonInputsPositional("Intake", "Extending Motor");

        double timeOfFlightDistMeters = 0.0;
        boolean isConnected = false;
        double currentDraw = 0.0;



    }

    public default void setIntakeMotorVolts(double volts){}

    public default void setExtendingMotorVolts(double volts){}

    public default void updateInputs(IntakeIOInputs inputs){}


}
