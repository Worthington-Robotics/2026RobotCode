package frc.WorBots.subsystems.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.WorBots.CanIDs;

public class TurretIOTalon implements TurretIO{
    //TODO change motor controller type to talonFX
    private Talon turretMotor;
    private Talon turretAngle;

    public TurretIOTalon(){
        turretMotor = new Talon(CanIDs.TURRET_ID);
        turretAngle = new Talon(CanIDs.TURRET_ENCODER_ID);

        //TODO use hardware utils to do this
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        //TODO implement TurretIOInputs
    }

}
