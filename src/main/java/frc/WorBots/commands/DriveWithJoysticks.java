package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.RobotContainer;
import frc.WorBots.subsystems.drive.Drive;

public class DriveWithJoysticks extends Command {
  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Boolean> slowSupplier;

  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier, Supplier<Boolean> slowSupplier){
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.slowSupplier = slowSupplier;
  }

  @Override
  public void initialize(){
    RobotContainer.driveController.reset();
  }

  @Override
  public void execute(){
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightX = rightXSupplier.get();

    if(slowSupplier.get()){
      leftX *= Constants.DRIVE_SLOW_MULTIPLIER;
      leftY *= Constants.DRIVE_SLOW_MULTIPLIER;
      rightX *= Constants.DRIVE_SLOW_MULTIPLIER;
    }

    RobotContainer.driveController.drive(drive, -leftY, leftX, rightX);
  }

  @Override
  public void end(boolean interupted){
    drive.stop();
  }
    
}
