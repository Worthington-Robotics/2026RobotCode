package frc.WorBots.subsystems.intake;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.intake.IntakeIO.IntakeIOInputs;
//import frc.WorBots.util.debug.StatusPage;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    //Whether or not the intake has fuel within it
    private boolean hasFuel = false;

    //The setpoint voltage for the intake
    private double setPointVoltage = 0.0;

    private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private final NetworkTable intakeTable = instance.getTable("Intake");
    private final DoublePublisher setpointPub =
        intakeTable.getDoubleTopic("Setpoint Volts").publish();
    private final BooleanPublisher hasGamePiecePub =
        intakeTable.getBooleanTopic("Has Game Piece").publish();
    private final DoublePublisher currentDrawPub =
        intakeTable.getDoubleTopic("Current Draw").publish();

    public Intake(IntakeIO io){
        this.io = io;
        //StatusPage.reportStatus(StatusPage.INTAKE_SUBSYSTEM, true);
    }

    
    
    


}
