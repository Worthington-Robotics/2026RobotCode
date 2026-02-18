package frc.WorBots.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.climber.Climber.climberControlMode;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;

public class ClimberIOTalon implements ClimberIO{
  TalonFX climbMotor = new TalonFX(0);
  TalonSignalsPositional climbMotorSignals;

  private final TunableProfiledPIDController climberController = 
    new TunableProfiledPIDController("climber","Climber PID controller");

  private final SimpleMotorFeedforward climberFeedforward = new SimpleMotorFeedforward(0, 0);

  private double position;
  private climberControlMode mode = climberControlMode.disabled;

  public ClimberIOTalon(){
    climbMotorSignals = new TalonSignalsPositional(climbMotor);
    //TODO add gains
    climberController.setGains(0,0 ,0 );
    climberController.setConstraints(Constants.DRIVE_MAX_ROTATIONAL_VELOCITY,Constants.DRIVE_MAX_ACCELERATION);
  }

  public void setVolts(double volts){
    if (mode == climberControlMode.voltage){
      climbMotorSignals.setVoltage(climbMotor, volts, 10);
    }
    
  }

  public void setPosition(double pos){
    position = pos;
    climberController.pid.setGoal(position);
  }

  @Override
  public void setControlMode(climberControlMode controlMode){
    mode = controlMode;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    
    if (mode != climberControlMode.disabled){
      if (mode == climberControlMode.position){
        double feedback = climberController.pid.calculate(position);
        double feedforward = climberFeedforward.calculate(climberController.pid.getSetpoint().velocity);
        setVolts(feedback + feedforward);
      } 
    } else {
      climbMotorSignals.setVoltage(climbMotor, 0.0, 0.0);
    }
  }
}