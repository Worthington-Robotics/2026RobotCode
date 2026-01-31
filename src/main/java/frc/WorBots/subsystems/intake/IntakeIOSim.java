package frc.WorBots.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.WorBots.Constants;

//import frc.WorBots.util.RobotSimulator;

public class IntakeIOSim implements IntakeIO{
    private FlywheelSim intakeSim;
    private FlywheelSim extendingSim;


    public IntakeIOSim(){
        intakeSim = new FlywheelSim(null, null, null);
        extendingSim = new FlywheelSim(null, null, null);
    }
}
