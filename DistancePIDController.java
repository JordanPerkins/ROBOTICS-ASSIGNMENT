import ShefRobot.*;
import java.util.concurrent.TimeUnit;

public class DistancePIDController {

    private static Robot robot = new Robot("dia-lego-c7");
    private static Motor leftMotor = robot.getLargeMotor(Motor.Port.C);
    private static Motor rightMotor = robot.getLargeMotor(Motor.Port.D);
    private static UltrasonicSensor sensor = robot.getUltrasonicSensor(Sensor.Port.S2);

    private static float Kp = 1;
    private static float Ki = 0;
    private static float Kd = 0;

    private static void pid() {
        float target = 0.50f;
        float integral = 0;
        float lastError = 0;
        float derivative = 0;
        while (true) {
            float value = sensor.getDistance();
            System.out.println(value);
            float error = value - target;
            integral = integral + error;
            derivative = error - lastError;
            float signal = (Kp*error + Ki*integral + Kd*derivative)*100;
            System.out.println(signal);
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
        long initialTime1 = System.nanoTime();
        while (true) {
            long timeChange = System.nanoTime() - initialTime;
            long timeChangeInSecs = TimeUnit.SECONDS.convert(timeChange, TimeUnit.NANOSECONDS);
            long timeChange1 = System.nanoTime() - initialTime1;
            long timeChangeInSecs1 = TimeUnit.SECONDS.convert(timeChange1, TimeUnit.NANOSECONDS);
            if (timeChangeInSecs >= 10.00) {
                initialTime = System.nanoTime();
                if (target == 0.50) {
                    target = 0.30f;
                } else {
                    target = 0.50f;
                }
            }
            if (timeChangeInSecs1 >= 30.00) {
                initialTime1 = System.nanoTime();
                Kp += 5;
                System.out.println("Kp value is now" + Kp);
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

    // Keep a set distance away from a wall to the right.
    private static void pid3() {
      float integral = 0;
      float lastError = 0;
      float derivative = 0;
      float target = 0.20f;
      int speed = 10; // This is a guess to be changed.
      while (true) {
        float value = sensor.getDistance();
        float error = value - target;
        integral = integral + error;
        derivative = error - lastError;
        float signal = Kp*error + Ki*integral + Kd*derivative;
        setMotorsTurn(speed,(int) signal);
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

    private static void setMotorsTurn(int speed, int steer) {
      // Correct for following a right hand side wall.
      if (steer > 0) {
        leftMotor.setSpeed(speed*steer);
        rightMotor.setSpeed(speed*steer);
        leftMotor.backward();
        rightMotor.forward();
      } else if (steer < 0) {
        leftMotor.setSpeed(speed*(-steer));
        rightMotor.setSpeed(speed*(-steer));
        leftMotor.forward();
        rightMotor.backward();
      } else {
        leftMotor.setSpeed(speed);
        rightMotor.setSpeed(speed);
        leftMotor.forward();
        rightMotor.forward();
      }
    }

    public static void main(String[] args) {
        pid();

        // Runtime.getRuntime().addShutdownHook(new Thread() {
        //   public void run() {
        //     leftMotor.stop();
        //     rightMotor.stop();
        //     robot.close();
        //   }
        // });
    }
}
