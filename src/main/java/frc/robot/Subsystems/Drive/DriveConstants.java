package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double MAX_SPEED_METERS_PER_SEC = 4.0;
  public static final double TRACK_WIDTH = Units.inchesToMeters(26.0);

  // Device CAN IDs
  public static final int PIGEON_CAN_ID = 9;
  public static final int LEFT_LEADER_CAN_ID = 1;
  public static final int LEFT_FOLLOWER_CAN_ID = 2;
  public static final int RIGHT_LEADER_CAN_ID = 3;
  public static final int RIGHT_FOLLOWER_CAN_ID = 4;

  // Motor configuration
  public static final int CURRENT_LIMIT = 60;
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);
  public static final double MOTOR_REDUCTION = 10.71;
  public static final boolean LEFT_INVERTED = false;
  public static final boolean RIGHT_INVERTED = true;
  public static final DCMotor GEARBOX = DCMotor.getCIM(2);

  // Velocity PID configuration
  public static final double REAL_KP = 0.0;
  public static final double REAL_KD = 0.0;
  public static final double REAL_KS = 0.0;
  public static final double REAL_KV = 0.1;

  public static final double SIM_KP = 0.05;
  public static final double SIM_KD = 0.0;
  public static final double SIM_KS = 0.0;
  public static final double SIM_KV = 0.227;

}