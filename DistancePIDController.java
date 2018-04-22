import ShefRobot.*;
import java.util.concurrent.TimeUnit;

public class DistancePIDController {

    private static Robot robot = new Robot();
    private static Motor leftMotor = robot.getLargeMotor(Motor.Port.A);
    private static Motor rightMotor = robot.getLargeMotor(Motor.Port.B);
    private static UltrasonicSensor sensor = robot.getUltrasonicSensor(Sensor.Port.S1);

    private static float Kp = 10;
    private static float Ki = 1;
    private static float Kd = 100;

    private static void pid() {
        float target = 0.50f;
        float integral = 0;
        float lastError = 0;
        float derivative = 0;
        while (true) {
            float value = sensor.getDistance();
            float error = value - target;
            integral = integral + error;
            derivative = error - lastError;
            float signal = Kp*error + Ki*integral + Kd*derivative;
            setMotors((int) signal);
            lastError = error;
        }
    }

    private static void pid1() {
        float integral = 0;
        float lastError = 0;
        float derivative = 0;
        float target = 0.50f;
        long initialTime = System.nanoTime();
        while (true) {
            long timeChange = System.nanoTime() - initialTime;
            long timeChangeInSecs = TimeUnit.SECONDS.convert(timeChange, TimeUnit.NANOSECONDS);
            if (timeChangeInSecs >= 10.00) {
                initialTime = System.nanoTime();
                if (target == 0.50) {
                    target = 0.30f;
                } else {
                    target = 0.50f;
                }
            }
            float value = sensor.getDistance();
            float error = value - target;
            integral = integral + error;
            derivative = error - lastError;
            float signal = Kp*error + Ki*integral + Kd*derivative;
            setMotors((int) signal);
            lastError = error;
        }
    }

    private static void setMotors(int speed) {
        if (speed >= 0) {
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            leftMotor.forward();
            rightMotor.forward();
        } else {
            leftMotor.setSpeed(-speed);
            rightMotor.setSpeed(-speed);
            leftMotor.backward();
            rightMotor.backward();
        }
    }

    public static void main(String[] args) {
        pid();
    }
}