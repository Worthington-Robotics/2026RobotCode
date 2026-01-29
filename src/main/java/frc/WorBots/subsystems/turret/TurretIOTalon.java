package frc.WorBots.subsystems.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.WorBots.CanIDs;

public class TurretIOTalon implements TurretIO{
    private Talon turretMotor;
    private Talon turretAngle

    public TurretIOTalon(){
        turretMotor = new Talon(CanIDs.TURRET_ID);
        turretAngle = new Talon(CanIDs.TURRET_ENCODER_ID);

        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        


    }

}
