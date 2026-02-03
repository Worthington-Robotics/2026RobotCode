package frc.WorBots.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.WorBots.Constants;

//import frc.WorBots.util.RobotSimulator;

public class IntakeIOSim implements IntakeIO{
    private FlywheelSim intakeSim = 
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), Constants.MOMENT_OF_INERTIA, Constants.INTAKE_INTAKE_GR), DCMotor.getKrakenX60(1));
    private SingleJointedArmSim extendingSim = 
        new SingleJointedArmSim(DCMotor.getKrakenX60(1), Constants.INTAKE_PIVOT_GR, Constants.INTAKE_PIVOT_GR, 0, 0, 0, false, 0, null);


    public IntakeIOSim(){}

    public void updateInputs(IntakeIOInputs inputs){
        //tell sim what inputs are 
        //tell update freq
        //update inputs
        intakeSim.update(Constants.ROBOT_PERIOD);
        inputs.isConnected = true;

        inputs.extendingMotor.velocityRadsPerSec = extendingSim.getVelocityRadPerSec();
        inputs.intakeMotor.velocityRadsPerSec = intakeSim.getAngularVelocityRadPerSec();

        inputs.extendingCurrent = extendingSim.getCurrentDrawAmps();
        inputs.intakeCurrent = intakeSim.getCurrentDrawAmps();
        
        //Figure this out later
        inputs.timeOfFlightDistMeters = 0.0;
    }

    
    public void setIntakeMotorVolts(double volts){
        intakeSim.setInputVoltage(volts);
    }

    public void setExtendingMotorVolts(double volts){
        extendingSim.setInputVoltage(volts);
    }
}
