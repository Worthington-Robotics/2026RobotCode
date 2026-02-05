package frc.WorBots.subsystems.intake;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;
import edu.wpi.first.wpilibj.DriverStation;
import frc.WorBots.Constants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    //TODO: Add enum in Hardware Utils so that able to switch b/w states
    //The setpoint voltage for the intake
    private double setPointVoltageIntake = 0.0;

    private double setPointVoltageExtending = 0.0;

    //All publishers
    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = instance.getTable("Intake");
    private final DoublePublisher setpointIntakePub =
        intakeTable.getDoubleTopic("Intake Setpoint Volts").publish();
    private final DoublePublisher setpointExtendingPub = 
        intakeTable.getDoubleTopic("Extending Setpoint Volts").publish();

    private final DoublePublisher currentDrawIntakePub =
        intakeTable.getDoubleTopic("Intake Current Draw").publish();
    private final DoublePublisher currentDrawExtendingPub = 
        intakeTable.getDoubleTopic("Extending Current Draw").publish(null);


    
    //Constructor 
    public Intake(IntakeIO io){
        this.io = io;
        //StatusPage.reportStatus(StatusPage.INTAKE_SUBSYSTEM, true);
    }

    //The periodic function
    @Override 
    public void periodic(){
        io.updateInputs(inputs);
        
    

        if (inputs.intakeMotor.temperatureCelsius > Constants.INTAKE_MAX_TEMP || DriverStation.isDisabled()|| inputs.extendingMotor.temperatureCelsius > Constants.INTAKE_MAX_TEMP){
            setPointVoltageIntake = 0.0;
        
        }

       /*  
        StatusPage.reportStatus(
            StatusPage.INTAKE_CONNECTED,
            inputs.isConnected && inputs.motor.temperatureCelsius <= Constants.INTAKE_MAX_TEMP); */

        //Setting the voltages of the motors    
        io.setIntakeMotorVolts(setPointVoltageIntake);

        io.setExtendingMotorVolts(setPointVoltageExtending);

        inputs.intakeMotor.publish();
        inputs.extendingMotor.publish();
        setpointIntakePub.set(setPointVoltageIntake);
        setpointExtendingPub.set(setPointVoltageExtending);
    
        currentDrawIntakePub.set(inputs.intakeCurrent);
        currentDrawExtendingPub.set(inputs.extendingCurrent);

    }

    //Getters and Setters

    public double getSetPointVoltageIntake(){
        return setPointVoltageIntake;
    }

    public double getSetPointVoltageExtending(){
        return setPointVoltageExtending;
    }


    public void setVoltsIntake(double voltage){
        setPointVoltageIntake = voltage;
        
    }

    public void setVoltsExtending(double voltage){
        setPointVoltageExtending = voltage;
    }



}
