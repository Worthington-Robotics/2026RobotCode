package frc.WorBots.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;

public class ClimberIOTalon implements ClimberIO{
  TalonFX climbMotor = new TalonFX(0);
  TalonSignalsPositional climbMotorSignals;

  private final TunableProfiledPIDController climberController = 
    new TunableProfiledPIDController("climber","Climber PID controller");

  private final SimpleMotorFeedforward climberFeedforward = new SimpleMotorFeedforward(0, 0);
  private double position;

  public ClimberIOTalon(){
    climbMotorSignals = new TalonSignalsPositional(climbMotor);
    climberController.setGains(position, position, position);
    climberController.setConstraints(0,0);
  }

  public void setVolts(double volts){
    climbMotorSignals.setVoltage(climbMotor, volts, 10);
  }

  public void setPosition(double pos){
    position = pos;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
   climberController.pid.setGoal(position);
   double feedback = climberController.pid.calculate(position);
   double feedforward = climberFeedforward.calculate(climberController.pid.getSetpoint().velocity);
   setVolts(feedback + feedforward);

   
  }
}