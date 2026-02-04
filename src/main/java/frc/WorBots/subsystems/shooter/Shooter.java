package frc.WorBots.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.shooter.ShooterIO.ShooterIOInputs;


public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputs inputs = new ShooterIOInputs();
    private boolean hasFuel = false;
    private double feederWheelVolts = 0.0;

    //Publish the values etc
}
