package frc.WorBots.subsystems.turret;

import java.lang.invoke.VarHandle.VarHandleDesc;
import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.turret.Turret.turretControlMode;
import frc.WorBots.subsystems.turret.TurretIO.TurretIOInputs;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.control.DutyCycleEncoderFilter;

public class TurretIOTalon {
    

    //electronics
  private TalonFX turretMotor;
  private CANcoder turretAbsEncoder; 
  private TurretIOInputs turretInputs;
  private double fusEncoderOffset;
  private boolean shouldReadAbsEncoder = true;
    

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
    turretAbsEncoder = new CANcoder(0);

    turretAbsEncoderSignal = turretAbsEncoder.getAbsolutePosition();
    turretRelEncoderSignal = turretMotor.getRotorPosition();
    turretMotorSignal = turretMotor.getPosition();
    motorCurrentSignal = turretMotor.getSupplyCurrent();
        
        
    turretAbsEncoderSignal.setUpdateFrequency(0.0);
    turretRelEncoderSignal.setUpdateFrequency(0.0);
    turretMotorSignal.setUpdateFrequency(0.0);
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

    
    public void updateInputs(){
      turretMotorSignal.refresh();
      turretAbsEncoderSignal.refresh();
      turretRelEncoderSignal.refresh();
      motorCurrentSignal.refresh();
      double volts = 0.0;
       
      if(turretInputs.controlMode == turretControlMode.Disabled){
        volts = 0.0;
      }

      if(turretInputs.controlMode == turretControlMode.Voltage){
        
        volts = turretInputs.debugVoltage;
        MathUtil.clamp(volts, -5, 5);
        turretMotor.setVoltage(volts);
      }

      if(turretInputs.controlMode == turretControlMode.Position){

        final double feedback = turretFeedBack.calculate(turretInputs.turretFusedAngle, turretInputs.goalAngle); //TODO check if this is right way to create PID controller
        final double feedforward = turretFeedForward.calculate(turretFeedBack.getSetpoint().velocity);  //send feedback to feedforward to get the velocity for feedforward

        volts = feedback + feedforward;

        MathUtil.clamp(volts, -5, 5);
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

          
          
          turretInputs.turretFusedAngle = MathUtil.angleModulus(relReading + fusEncoderOffset);

          } 

        }

      
     turretInputs.absEncoderConnected = turretAbsEncoder.isConnected();

    }
      
         


    
  

      
      
    


  
 

}
// 1. Make Abs Encoder Optional COMPLETE
// 2. Finish setPoint Method 
// 3. Create setVoltage 
// 4. Create resetZero 
// 5. Create getPosition 
// 6. Create boolean atSetPoint 