package frc.robot.Subsystems.Drive;

import static frc.robot.Subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71, KitbotWheelSize.kSixInch, null);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private PIDController leftPID = new PIDController(SIM_KP, 0.0, SIM_KD);
  private PIDController rightPID = new PIDController(SIM_KP, 0.0, SIM_KD);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts =
          leftFFVolts + leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / WHEEL_RADIUS_METERS);
      rightAppliedVolts =
          rightFFVolts
              + rightPID.calculate(sim.getRightVelocityMetersPerSecond() / WHEEL_RADIUS_METERS);
    }

    // Update simulation state
    sim.setInputs(
        MathUtil.clamp(leftAppliedVolts, -12.0, 12.0),
        MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.leftPositionRad = sim.getLeftPositionMeters() / WHEEL_RADIUS_METERS;
    inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / WHEEL_RADIUS_METERS;
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};

    inputs.rightPositionRad = sim.getRightPositionMeters() / WHEEL_RADIUS_METERS;
    inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / WHEEL_RADIUS_METERS;
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
    leftPID.setSetpoint(leftRadPerSec);
    rightPID.setSetpoint(rightRadPerSec);
  }
}