package frc.robot.util;

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

}
