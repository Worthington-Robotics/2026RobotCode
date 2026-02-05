package frc.WorBots.subsystems.intake;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    //TODO: Add enum in Hardware Utils so that able to switch b/w states

    //The setpoint voltages for both the intaking and extending motors
    private double setPointVoltageIntake = 0.0;

    private double setPointVoltageExtending = 0.0;

    //Current draw and setpoint publishers.
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


     
    public Intake(IntakeIO io){
        this.io = io;
        StatusPage.reportStatus(StatusPage.INTAKE_SUBSYSTEM, true);
    }

    /***
     * Updates every period, reports Status, checks if the motors are too hot,
     * sets the voltages supplied to each motor, and publishes the motors, current draw,
     * and setpoints voltages.
     */
    @Override 
    public void periodic(){
        io.updateInputs(inputs);
        
    
        //If the heat exceeds the max, turn it off.
        if (inputs.intakeMotor.temperatureCelsius > Constants.INTAKE_MAX_TEMP || DriverStation.isDisabled()|| inputs.extendingMotor.temperatureCelsius > Constants.INTAKE_MAX_TEMP){
            setPointVoltageIntake = 0.0;
        
        }

       
        StatusPage.reportStatus(
            StatusPage.INTAKE_CONNECTED,
            inputs.isConnected && inputs.intakeMotor.temperatureCelsius <= Constants.INTAKE_MAX_TEMP); 


        //Setting the voltages of the motors    
        io.setIntakeMotorVolts(setPointVoltageIntake);

        io.setExtendingMotorVolts(setPointVoltageExtending);

        //Publishing the motor signals.
        inputs.intakeMotor.publish();
        inputs.extendingMotor.publish();

        //Publishing the setpoint voltage for extending and intaking motors.
        setpointIntakePub.set(setPointVoltageIntake);
        setpointExtendingPub.set(setPointVoltageExtending);
    
        //Publishing the current draw for extending and intaking motors.
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
