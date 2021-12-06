package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.Timer;

public class TurnGyroPID extends CommandBase 
{
  private final DriveTrain _driveTrain;
  
  private  double _Distance = 0;
  private  double _Speed = 0;
  private double _Angle = 0;

  private double kP = .65;
  private double kI = 1;
  private double kD = 1;

  private double P = 1;
  private double I = 1;
  private double D = 1;

  private double error = 0;
  private double errorSum = 0;
  private double lastError = 0;

  private int cont=1;

  private Timer _Timer;
  private double timeStamp;

  public TurnGyroPID(DriveTrain dt, double angle) 
  {
    _driveTrain = dt;

    _Angle = angle;

    //positive is right negative us left
    if(angle>=0){
      cont=1;
    }
    else{
      cont=-1;
    }
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
    _driveTrain.resetN();
  }

  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Angles", _driveTrain.getAngle());
    error = _Angle - _driveTrain.getAngle();
    error = (error/_Angle);
    _Speed = error*kP;
          
    if (_Speed > 0.7){
      _Speed = 0.7;

    }

    if (_Speed < 0.25){
      _Speed = 0.25;
    }

    _driveTrain.tankDrive(_Speed* cont, -_Speed*cont);
  }

  @Override
  public void end(boolean interrupted) 
  {
    System.out.println(_driveTrain.getAngle());
      _driveTrain.tankDrive(0, 0);
      _driveTrain.resetEncoders();
      _driveTrain.resetN();
  }

  @Override
  public boolean isFinished() 
  {
      return Math.abs(_driveTrain.getAngle()) >= _Angle;
  }
}