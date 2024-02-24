package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveUtil {

    public static double getAngle(double xAxis, double yAxis, double yaw) {

        if (Math.abs(xAxis) < 0.1 && Math.abs(yAxis) < 0.1) {
            return yaw;
        }

        double controllerAngle = Math.atan2(yAxis,  xAxis) * 180 / Math.PI;

        if (controllerAngle > -90) {
            controllerAngle = -controllerAngle + 90;
        }

        else controllerAngle = -controllerAngle - 270;

        return controllerAngle;

    }

    public static double getDisplacement(double target, double current) {
        double ogD = target - current;

        double displacement = ((ogD + 180) % 360) - 180;

        if (Math.abs(displacement) > 90) {
            displacement = -(180 - Math.abs(displacement)) * Math.signum(displacement);
        }

        return displacement;
    }

    public static boolean handleDrive(double xAxis, double yAxis) {

        double x = xAxis;
        double y = yAxis;

        boolean reBool = false;

        if (Math.abs(x) > 0.1)
            reBool = true;
        if (Math.abs(y) > 0.1)
            reBool = true;
        
        return reBool;

    }
    
    public static boolean handleTurn(double axis) {

        boolean reBool = false;

        if (Math.abs(axis) > 0.1)
            reBool = true;
        
        return reBool;

    }

    public static void moveSwerveModule(SwerveModuleState state, SwerveModule mod) {
        SmartDashboard.putNumber(mod.m_name + " state speed: ", state.speedMetersPerSecond);
        double driveSpeed = mod.getDriveMotorSpeed(state.speedMetersPerSecond * DriveConstants.driveVelConvFactor * 3.18 * 0.5);
        double goalAngle = state.angle.getDegrees() + 180;
        double ogD = goalAngle - mod.getTurnMotorPosition();
        double d = SwerveUtil.getDisplacement(goalAngle, mod.getTurnMotorPosition());
        if (!(Math.abs(goalAngle - mod.getTurnMotorPosition()) < 2.5)) {
            double turnVoltage = mod.getTurnMotorSpeed(d);
            double newDriveSpeed = Math.abs(ogD) > 90 && Math.abs(ogD) < 270 ? -driveSpeed : driveSpeed;
            mod.move(newDriveSpeed, turnVoltage);
        } else {
            double turnSpeed = 0;
            mod.move(driveSpeed, turnSpeed);
        }
    }
}
