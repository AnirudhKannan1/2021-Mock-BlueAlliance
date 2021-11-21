package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Timer;

public class TurnGyroPID extends CommandBase 
{
  private final DriveTrain _driveTrain;
  
  private  double _Distance = 0;
  private  double _Speed = 0;
  private double _Angle = 0;

  private double kP = 1;
  private double kI = 1;
  private double kD = 1;

  private double P = 1;
  private double I = 1;
  private double D = 1;

  private double error = 0;
  private double errorSum = 0;
  private double lastError = 0;

  private Timer _Timer;
  private double timeStamp;

  public TurnGyroPID(DriveTrain dt, double angle) 
  {
    _driveTrain = dt;

    angle = _Angle;

    timeStamp = 0;
    _Timer = new Timer();

    addRequirements(_driveTrain);
   }

  @Override
  public void initialize() 
  {
    _Timer.reset();
    _Timer.start();
    _driveTrain.resetEncoders();
  }

  @Override
  public void execute() 
  {
    double _receivedTime = _Timer.get();
    error = _Angle - _driveTrain.getAngle();
    errorSum += (error * (_receivedTime - timeStamp));
          
    P = error;
    I = errorSum; 
    D = (error - lastError) / _receivedTime - timeStamp;
    timeStamp = _receivedTime;
    lastError = error;

    _Speed = kP * P + kI * I + kD * D;

    _driveTrain.tankDrive(_Speed,0);
  }

  @Override
  public void end(boolean interrupted) 
  {
    System.out.println(_driveTrain.getAngle());
      _driveTrain.tankDrive(0, 0);
      _driveTrain.resetEncoders();
  }

  @Override
  public boolean isFinished() 
  {
      return _driveTrain.getAngle() >= _Angle;
  }
}