// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
  private final Arm arm = Arm.getInstance();
  Timer timer;
  double position;

  


  @Override
  public void initialize() {

    this.arm.putArmInPoseMM(position);

  }
    


  @Override
  public void execute() {
    

  }


  @Override
  public void end(boolean interrupted) {
    arm.moveArm(0.0);

  }


  @Override
  public boolean isFinished() {
    if ((arm.getPosArm() > this.position - Constants.ArmConstants.Differances)
        && (arm.getPosArm() < this.position + Constants.ArmConstants.Differances)){
      return (this.timer.hasElapsed(Constants.ArmConstants.Time));
    }
    else{
      this.timer.restart();
      return false;
    }

  }
}
