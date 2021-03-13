package lib;

import java.lang.reflect.Method;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryCommandGenerator {
    static ArrayList<RamseteCommand> RamseteCommandList;

    public static ArrayList<RamseteCommand> getTrajectoryCommand(ArrayList<Trajectory> exampleTrajectory,
            DriveSubsystem m_robotDrive, PIDController leftPIDController, PIDController rightPIDController) {

        for (Trajectory trajectory : exampleTrajectory) {
            RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_robotDrive::getPose,
                    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                            DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds, leftPIDController,
                    rightPIDController,
                    // RamseteCommand passes volts to the callback
                    m_robotDrive::tankDriveVolts, m_robotDrive);

            RamseteCommandList.add(ramseteCommand);
                }
        

        return RamseteCommandList;
    }
}
