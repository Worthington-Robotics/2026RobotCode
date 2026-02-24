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
    

    //electronics
  private TalonFX turretMotor;
  private CANcoder turretAbsEncoder; 
  private TurretIOInputs turretInputs;
  private double fusEncoderOffset;
  private boolean shouldReadAbsEncoder = true;
  private double setpointPosition;
  turretControlMode controlMode = turretControlMode.Disabled;
  public static final double MIN_ANGLE = Units.degreesToRadians(-270.0);
  public static final double MAX_ANGLE = Units.degreesToRadians(270.0);
  public static final double MIN_VOLTAGE = 0.0;
  public static final double MAX_VOLTAGE = 10.0;
    

  private final StatusSignal<Angle> turretAbsEncoderSignal;
  private final StatusSignal<Angle> turretRelEncoderSignal;
  private final StatusSignal<Angle> turretMotorSignal;

  private final StatusSignal<Current> motorCurrentSignal;
  private final TalonSignalsPositional motorSignal;


  public final SimpleMotorFeedforward turretFeedForward = new SimpleMotorFeedforward(0.0, 0.0);
  public  ProfiledPIDController turretFeedBack = new ProfiledPIDController(2, 0, 0,
    new TrapezoidProfile.Constraints(0.0, 0.0));


  public TurretIOTalon(){

    turretInputs = new TurretIOInputs();
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
      turretInputs.controlMode = turretControlMode.Voltage;
      turretInputs.debugVoltage = volts;
      
    }

    public void resetZero(TurretIOInputs inputs, double position) {
      fusEncoderOffset += position - turretInputs.turretFusedAngle;
    }

    
    public void updateInputs(TurretIOInputs inputs){
      turretMotorSignal.refresh();
      turretAbsEncoderSignal.refresh();
      turretRelEncoderSignal.refresh();
      motorCurrentSignal.refresh();
      double volts = 0.0;

      if(turretInputs.controlMode == turretControlMode.Voltage){
        
        volts = turretInputs.debugVoltage;
        MathUtil.clamp(volts, MIN_VOLTAGE, MAX_VOLTAGE);
        turretMotor.setVoltage(volts);
      }

      if(turretInputs.controlMode == turretControlMode.Position){

        final double feedback = turretFeedBack.calculate(turretInputs.turretFusedAngle, turretInputs.goalAngle); 
        final double feedforward = turretFeedForward.calculate(turretFeedBack.getSetpoint().velocity);  //send feedback to feedforward to get the velocity for feedforward

        volts = feedback + feedforward;

        MathUtil.clamp(volts, MIN_VOLTAGE, MAX_VOLTAGE);
        turretMotor.setVoltage(volts);
        
        final double relReading = turretRelEncoderSignal.getValue().in(edu.wpi.first.units.Units.Radians);
            turretInputs.turretRelAngle = relReading;

        final Optional<Double> absReading = Optional.ofNullable(turretAbsEncoderSignal.getValue())
            .map(
              reading -> {
                return MathUtil.angleModulus(
                  reading.in(edu.wpi.first.units.Units.Radians));
              }
            );
        
        
          if(absReading.isPresent()){
          turretInputs.turretAbsAngle = absReading.get();
        }

        if (shouldReadAbsEncoder) {
          if(absReading.isPresent()){
            fusEncoderOffset = absReading.get() - relReading;
            shouldReadAbsEncoder = false;

          } else if (absReading.isEmpty()){
            fusEncoderOffset = 0.0;
          }          
          turretInputs.turretFusedAngle = relReading + fusEncoderOffset;

          } 

        }

      
     turretInputs.absEncoderConnected = turretAbsEncoder.isConnected();

    }

  private double clampSetpoint(double setpoint) {
      return MathUtil.clamp(setpoint, MIN_ANGLE, MAX_ANGLE);
  }

  public void setPosition(double positionRads){
   
    positionRads = clampSetpoint(positionRads);
    if (controlMode != controlMode.Position) {
      turretFeedBack.reset(turretInputs.turretFusedAngle);
    }

    if (positionRads != setpointPosition) {
      //TODO command the pid in turretIOTalon
      turretFeedBack.setGoal(positionRads);
    }
   
    setpointPosition = positionRads;
    controlMode = turretControlMode.Position;
      
  }

  public boolean atSetPoint() {
    return turretFeedBack.atGoal();
  }

         


    
  

      
      
    


  
 

}
// 1. Make Abs Encoder Optional COMPLETE
// 2. Finish setPoint Method 
// 3. Create setVoltage 
// 4. Create resetZero 
// 5. Create getPosition 
// 6. Create boolean atSetPoint 