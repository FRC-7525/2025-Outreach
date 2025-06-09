package frc.robot.Subsystems.Drive;

import static frc.robot.GlobalConstants.*;
import static frc.robot.GlobalConstants.Controllers.*; 
import static frc.robot.Subsystems.Drive.DriveConstants.*; 
import static frc.robot.Subsystems.Drive.DriveStates.*; 

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drive.DriveIO.DriveIOInputs;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

public class Drive extends Subsystem<DriveStates> {

    private static Drive instance; 

    private DriveIO io; 
    private DriveIOInputsAutoLogged inputs; 
    public boolean slow; 

    private final Field2d field; 

    public static Drive getInstance() {
        if (instance == null) {
            DriveIO driveIO = switch (ROBOT_MODE) {
                case REAL -> new DriveIOReal(); 
                case SIM -> new DriveIOSim(); 
                case TESTING -> new DriveIOReal(); 
            };
            instance = new Drive(driveIO);
        }
        return instance;
    } 

    private Drive(DriveIO io) {
        super("Drive", DriveStates.FIELD_RELATIVE); 
        this.io = io; 

        field = new Field2d(); 
        inputs = new DriveIOInputsAutoLogged(); 
    }

    @Override
    public void runState() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);
        logOutputs(); 
        setSpeed(); 

        getState().driveRobot();

        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);
    }

    public void logOutputs(SwerveDrive state) {
        Logger.recordOutput(SUBSYSTEM_NAME + "/Pose", getPose());
		Logger.recordOutput(SUBSYSTEM_NAME + "/Drive State", getState());
    }

    public void establishTriggers() {
        addTrigger(FIELD_RELATIVE, ROBOT_RELATIVE, DRIVER_CONTROLLER::getRightBumperButtonPressed);
        addTrigger(ROBOT_RELATIVE, FIELD_RELATIVE, DRIVER_CONTROLLER::getRightBumperButtonPressed); 

        addRunnableTrigger(() -> {io.zeroGyro();}, DRIVER_CONTROLLER::getLeftBumperButtonPressed);
        addRunnableTrigger(() ->{slow = true;}, DRIVER_CONTROLLER::getRightTriggerAxis > 50);
    }

    public void setSpeed() {
        io.setSpeed();
    }

    public void zeroGyro() {
        io.zeroGyro();
    }

    //stuff 
    public Pose2d getPose() {
        return io.getDrive().getPose(); 
    }

    public SwerveDrive getDrive() {
        return io.getDrive(); 
    }

    public ChassisSpeeds getSwerveInputs() {
        return io.getSwerveInputs(); 
    }

}