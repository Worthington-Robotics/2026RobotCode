package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.RobotContainer;
import frc.WorBots.subsystems.drive.Drive;

public class DriveWithJoysticks extends Command {
  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;

  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier, Supplier<Double> rightXSupplier){
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
  }

  @Override
  public void initialize(){
    RobotContainer.driveController.reset();
  }

  @Override
  public void execute(){
    final double leftX = leftXSupplier.get();
    final double leftY = leftYSupplier.get();
    final double rightX = rightXSupplier.get();

    RobotContainer.driveController.drive(drive, leftX, leftY, rightX);
  }

  @Override
  public void end(boolean interupted){
    drive.stop();
  }
    
}
