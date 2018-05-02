import ShefRobot.*;
import java.util.concurrent.TimeUnit;

public class DistancePIDController {

    private static Robot robot = new Robot("dia-lego-c7");
    private static Motor leftMotor = robot.getLargeMotor(Motor.Port.C);
    private static Motor rightMotor = robot.getLargeMotor(Motor.Port.D);
    private static UltrasonicSensor sensorR = robot.getUltrasonicSensor(Sensor.Port.S1);
    private static TouchSensor touch = robot.getTouchSensor(Sensor.Port.S4);
    private static UltrasonicSensor sensorL = robot.getUltrasonicSensor(Sensor.Port.S2);

    private static float Kp = 1f;
    private static float Ki = 0;
    private static float Kd = 0;

    private static void pid() {
        float target = 50f;
        float integral = 0;
        float lastError = 0;
        float derivative = 0;
        float error = 0;
        while (true) {
            float value = sensorR.getDistance()*100;
            if (value != Float.POSITIVE_INFINITY) {
              error = value - target;
            }
            System.out.println(value);
            integral = integral + error;
            derivative = error - lastError;
            float signal = (Kp*error + Ki*integral + Kd*derivative);
            System.out.println("KP WOULD BE"+(signal/error));
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
        long initialTime1 = System.nanoTime();
        float error = 0;
        while (true) {
            long timeChange = System.nanoTime() - initialTime;
            long timeChangeInSecs = TimeUnit.SECONDS.convert(timeChange, TimeUnit.NANOSECONDS);
            long timeChange1 = System.nanoTime() - initialTime1;
            long timeChangeInSecs1 = TimeUnit.SECONDS.convert(timeChange1, TimeUnit.NANOSECONDS);
            if (timeChangeInSecs >= 10.00) {
                initialTime = System.nanoTime();
                if (target == 50) {
                    target = 30f;
                } else {
                    target = 50f;
                }
            }
            float value = sensorR.getDistance()*100;
            if (value > 125) {
              leftMotor.stop();
              rightMotor.stop();
              robot.close();
              System.exit(0);
            }
            //if (timeChangeInSecs1 >= 30.00) {
            //    initialTime1 = System.nanoTime();
            //    System.out.println("Kp value is now" + Kp);
            //}
            if (value != Float.POSITIVE_INFINITY) {
              error = value - target;
            }
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
      int speed = 75; // This is a guess to be changed.
      while (true) {
        checkClose();
        float value = sensorL.getDistance()*100;
        float error = value - target;
        System.out.println("-------------------------------------------------");
        if (value > 125) {
          System.out.println("error");
          error = lastError;
        }
        integral = integral + error;
        derivative = error - lastError;
        float signal = Kp*error + Ki*integral + Kd*derivative;
        System.out.println(value);
        System.out.println(signal);
        setMotorsTurn(speed,(int) signal);
        lastError = error;
      }
    }

    private static void checkClose() {
      if (touch.isTouched()) {
        leftMotor.stop();
        rightMotor.stop();
        robot.close();
        System.exit(0);
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

    private static void pidMaze() {
      float integral = 0;
      float lastError = 0;
      float derivative = 0;
      float target = 100f;
      int speed = 75; // To be altered
      float errorDistance = 10f; // To be altered
      while (true) {
        checkClose();
        float valueR = sensorR.getDistance()*100;
        float valueL = sensorL.getDistance()*100;
        System.out.println("-------------------------------------------------");
        System.out.println(valueR);
        System.out.println(valueL);

        while (valueR > target+errorDistance) {
          System.out.print("Right Error");
          checkClose();
          float value = sensorL.getDistance()*100;
          float error = value - target;
          if (value > 60) {
            error = lastError;
          }
          integral = integral + error;
          derivative = error - lastError;
          float signal = Kp*error + Ki*integral + Kd*derivative;
          setMotorsTurn(speed,(int) signal);
          lastError = error;
          valueR = sensorR.getDistance()*100;
        }

        while (valueL > target+errorDistance) {
          System.out.print("Left Error");
          checkClose();
          float value = sensorR.getDistance()*100;
          float error = value - target;
          if (value > 60) {
            System.out.println("error");
            error = lastError;
          }
          integral = integral + error;
          derivative = error - lastError;
          float signal = Kp*error + Ki*integral + Kd*derivative;
          setMotorsTurn(speed,(int) (-signal));
          lastError = error;
          valueL = sensorL.getDistance()*100;
        }

        float error = valueL-valueR;
        integral = integral + error;
        derivative = error - lastError;
        float signal = Kp*error + Ki*integral + Kd*derivative;
        System.out.println(signal);
        setMotorsTurn(speed,(int) signal);
        lastError = error;
        target = ((valueR+valueL)/2.0f);
      }
    }

    private static void setMotorsTurn(int speed, int steer) {
      int rightSpeed = speed+steer;
      int leftSpeed = speed-steer;
      if (rightSpeed < 0) {
        rightMotor.setSpeed(-rightSpeed);
        leftMotor.setSpeed(leftSpeed);
        rightMotor.backward();
        leftMotor.forward();
      } else if (leftSpeed < 0) {
        rightMotor.setSpeed(rightSpeed);
        leftMotor.setSpeed(-leftSpeed);
        rightMotor.forward();
        leftMotor.backward();
      } else {
        rightMotor.setSpeed(rightSpeed);
        leftMotor.setSpeed(leftSpeed);
        rightMotor.forward();
        leftMotor.forward();
      }
  }

    public static void main(String[] args) {
        pidMaze();

        // Runtime.getRuntime().addShutdownHook(new Thread() {
        //   public void run() {
        //     leftMotor.stop();
        //     rightMotor.stop();
        //     robot.close();
        //   }
        // });
    }
}
