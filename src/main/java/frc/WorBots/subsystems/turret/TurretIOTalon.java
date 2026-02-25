package frc.WorBots.subsystems.turret;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.WorBots.CanIDs;
import frc.WorBots.subsystems.turret.Turret.turretControlMode;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.Constants;


public class TurretIOTalon implements TurretIO{
    

  //   //electronics
  private TalonFX turretMotor;
  private CANcoder turretAbsEncoder; 
  private TurretIOInputs inputs;

  private final StatusSignal<Angle> turretAbsEncoderSignal;
  private final StatusSignal<Angle> turretRelEncoderSignal;
  private final StatusSignal<Angle> turretMotorSignal;

  private final StatusSignal<Current> motorCurrentSignal;
  private final TalonSignalsPositional motorSignal;

  private double fusEncoderOffset;
  private boolean shouldReadAbsEncoder = true;

  public TurretIOTalon(){

    inputs = new TurretIOInputs();
    turretMotor = new TalonFX(CanIDs.TURRET_ID);

    turretAbsEncoder = new CANcoder(CanIDs.TURRET_ABS_ENCODER_ID);

    turretAbsEncoderSignal = turretAbsEncoder.getAbsolutePosition();
    turretRelEncoderSignal = turretMotor.getRotorPosition();
    turretMotorSignal = turretMotor.getPosition();
    motorCurrentSignal = turretMotor.getSupplyCurrent();
        
    turretAbsEncoderSignal.setUpdateFrequency(Constants.ROBOT_FREQUENCY);
    turretRelEncoderSignal.setUpdateFrequency(Constants.ROBOT_FREQUENCY);
    turretMotorSignal.setUpdateFrequency(Constants.ROBOT_FREQUENCY);
    turretMotor.optimizeBusUtilization();
        
    motorSignal = new TalonSignalsPositional(turretMotor);
        
    //TODO use hardware utils to do this
    turretMotor.setNeutralMode(NeutralModeValue.Brake);


    //TODO figure out how to implement the right time to wrap around 

  }


    
  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  //TODO figure out how we're implementing this
  //public void resetZero(TurretIOInputs inputs, double position) {
  //   fusEncoderOffset += position - turretInputs.turretFusedAngle;
  //}

    
  public void updateInputs(TurretIOInputs inputs){
    turretMotorSignal.refresh();
    turretAbsEncoderSignal.refresh();
    turretRelEncoderSignal.refresh();
    motorCurrentSignal.refresh();
        
    final double relReading = turretRelEncoderSignal.getValue().in(edu.wpi.first.units.Units.Radians);
    inputs.turretRelAngle = relReading;
  
    final Optional<Double> absReading = Optional.ofNullable(turretAbsEncoderSignal.getValue())
      .map(
        reading -> {
          return MathUtil.angleModulus(
            reading.in(edu.wpi.first.units.Units.Radians));
        }
      );
      
    //TODO take a look at this
    if(absReading.isPresent()){
      inputs.turretAbsAngle = absReading.get();
    }
  
    if (shouldReadAbsEncoder) {
        if(absReading.isPresent()){
            fusEncoderOffset = absReading.get() - relReading;
            shouldReadAbsEncoder = false;
  
        } else if (absReading.isEmpty()){
            fusEncoderOffset = 0.0;
        }
    } 

    inputs.turretFusedAngle = relReading + fusEncoderOffset; 
    inputs.absEncoderConnected = turretAbsEncoder.isConnected();
  }
} 


// 1. Make Abs Encoder Optional COMPLETE
// 2. Finish setPoint Method 
// 3. Create setVoltage 
// 4. Create resetZero 
// 5. Create getPosition 
// 6. Create boolean atSetPoint 