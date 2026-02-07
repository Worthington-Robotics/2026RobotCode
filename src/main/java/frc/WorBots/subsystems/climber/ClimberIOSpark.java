package frc.WorBots.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

public class ClimberIOSpark {
  SparkFlex climbMotor = new SparkFlex(0, SparkLowLevel.MotorType.kBrushless);
}