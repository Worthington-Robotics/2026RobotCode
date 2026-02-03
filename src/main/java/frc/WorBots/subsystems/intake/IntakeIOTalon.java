package frc.WorBots.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.util.HardwareUtils.OptimalStatusSignal;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.util.HardwareUtils.OptimalStatusSignal;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.Constants;
import frc.WorBots.CanIDs;

public class IntakeIOTalon implements IntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFX extendingMotor;

    private final TalonSignalsPositional intakeMotorSignals;
    private final TalonSignalsPositional extendingMotorSignals;

    private final OptimalStatusSignal<Current> intakeCurrentDrawSignal;
    private final OptimalStatusSignal<Current> extendingCurrentDrawSignal;

    private final LinearFilter tofFilter = LinearFilter.movingAverage(1);

    public IntakeIOTalon(){

        intakeMotor = new TalonFX(CanIDs.Main.INTAKE_MOTOR_ID, Constants.MAIN_CAN_BUS );
        extendingMotor = new TalonFX(CanIDs.Main.EXTENDING_MOTOR_ID, Constants.MAIN_CAN_BUS);


        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        HardwareUtils.setInverted(extendingMotor, false);
        extendingMotor.setNeutralMode(NeutralModeValue.Brake);
        HardwareUtils.setInverted(intakeMotor, false);

        intakeMotorSignals = new TalonSignalsPositional(intakeMotor);
        extendingMotorSignals = new TalonSignalsPositional(extendingMotor);




        intakeCurrentDrawSignal = 
            new OptimalStatusSignal<>(intakeMotor.getStatorCurrent(), Constants.ROBOT_PERIOD);
        extendingCurrentDrawSignal = 
            new OptimalStatusSignal<>(extendingMotor.getStatorCurrent(), Constants.ROBOT_PERIOD);

        HardwareUtils.setCurrentLimit(intakeMotor, 160);
        HardwareUtils.setCurrentLimit(extendingMotor, 160);    

        intakeMotor.optimizeBusUtilization();
        extendingMotor.optimizeBusUtilization();

    }

    public void updateInputs(IntakeIOInputs inputs){
        intakeMotorSignals.update(inputs.intakeMotor, intakeMotor);
        extendingMotorSignals.update(inputs.extendingMotor, extendingMotor);

        inputs.intakeCurrent = intakeCurrentDrawSignal.getValue().in(edu.wpi.first.units.Units.Amps);
        inputs.extendingCurrent = extendingCurrentDrawSignal.getValue().in(edu.wpi.first.units.Units.Amps);
    
    }

    //Need to actually figure out the max voltage
    public void setIntakeVolts(double volts){
        intakeMotorSignals.setVoltage(intakeMotor, volts, 10);
    }

    public void setExtendingMotorVolts(double volts){
        extendingMotorSignals.setVoltage(extendingMotor, volts, 10);
    }
    
}
