package frc.WorBots.subsystems.intake;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;


public interface IntakeIO {
    public static class IntakeIOInputs{ 
        
        TalonInputsPositional intakeMotor = new TalonInputsPositional("Intake", "Intake Motor");
        TalonInputsPositional extendingMotor = new TalonInputsPositional("Intake", "Extending Motor");

        
        boolean isConnected = false;
        double intakeCurrent;
        double extendingCurrent;



    }

    public default void setIntakeMotorVolts(double volts){}

    public default void setExtendingMotorVolts(double volts){}

    public default void updateInputs(IntakeIOInputs inputs){}


}
