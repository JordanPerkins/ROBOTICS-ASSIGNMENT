import ShefRobot.*;
import java.util.concurrent.TimeUnit;

public class DistancePIDController {

    private static Robot robot = new Robot("dia-lego-c7");
    private static Motor leftMotor = robot.getLargeMotor(Motor.Port.C);
    private static Motor rightMotor = robot.getLargeMotor(Motor.Port.D);
    private static UltrasonicSensor sensor = robot.getUltrasonicSensor(Sensor.Port.S3);

    private static float Kp = 1;
    private static float Ki = 0;
    private static float Kd = 0;

    private static void pid() {
        float target = 50f;
        float integral = 0;
        float lastError = 0;
        float derivative = 0;
        while (true) {
            float value = sensor.getDistance()*100;
            System.out.println(value);
            float error = value - target;
            integral = integral + error;
            derivative = error - lastError;
            float signal = (Kp*error + Ki*integral + Kd*derivative);
            System.out.println(signal);
            setMotors((int) signal);
            lastError = error;
        }
    }

    private static void pid1() {
        float integral = 0;
        float lastError = 0;
        float derivative = 0;
        float target = 50f;
        long initialTime = System.nanoTime();
        while (true) {
            long timeChange = System.nanoTime() - initialTime;
            long timeChangeInSecs = TimeUnit.SECONDS.convert(timeChange, TimeUnit.NANOSECONDS);
            if (timeChangeInSecs >= 10.00) {
                initialTime = System.nanoTime();
                if (target == 50) {
                    target = 30f;
                } else {
                    target = 50f;
                }
            }
            float value = sensor.getDistance()*100;
            System.out.println(value);
            if (value > 125) {
              leftMotor.stop();
              rightMotor.stop();
              robot.close();
              System.exit(0);
            }
            float error = value - target;
            integral = integral + error;
            derivative = error - lastError;
            float signal = (Kp*error + Ki*integral + Kd*derivative);
            setMotors((int) signal);
            lastError = error;
        }
    }

    // Keep a set distance away from a wall to the right.
    private static void pid3() {
      float integral = 0;
      float lastError = 0;
      float derivative = 0;
      float target = 20f;
      int speed = 50; // This is a guess to be changed.
      while (true) {
        float value = sensor.getDistance()*100;
        if (value > 125) {
          leftMotor.stop();
          rightMotor.stop();
          robot.close();
          System.exit(0);
        }
        float error = value - target;
        integral = integral + error;
        derivative = error - lastError;
        float signal = Kp*error + Ki*integral + Kd*derivative;
        System.out.println(speed+(-signal));
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
      rightMotor.setSpeed(speed+steer);
      leftMotor.setSpeed(speed-steer);
      rightMotor.forward();
      leftMotor.forward();
  }

    public static void main(String[] args) {
        pid3();

        // Runtime.getRuntime().addShutdownHook(new Thread() {
        //   public void run() {
        //     leftMotor.stop();
        //     rightMotor.stop();
        //     robot.close();
        //   }
        // });
    }
}
