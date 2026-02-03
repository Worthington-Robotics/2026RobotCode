package frc.WorBots.subsystems.turret;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.WorBots.CanIDs;

public class TurretIOTalon {
    //TODO change motor controller type to talonFX
    private TalonFX turretMotor;
    private TalonFX turretAngle;

    public TurretIOTalon(){
        turretMotor = new TalonFX(CanIDs.TURRET_ID);
        turretAngle = new TalonFX(CanIDs.TURRET_ENCODER_ID);

        //TODO use hardware utils to do this
        turretMotor.setNeutralMode(NeutralModeValue.Brake);

        //TODO implement TurretIOInputs
    }

}
