import ShefRobot.*;

/**
 *
 * @author sdn
 */
public class RobotSample {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {

        //Create a robot object to use and connect to it
        Robot myRobot = new Robot("dia-lego-c7");

        //The robot is made of components which are themselves objects.
        //Create references to them as useful shortcuts
        Motor leftMotor = myRobot.getLargeMotor(Motor.Port.B);
        Motor rightMotor = myRobot.getLargeMotor(Motor.Port.C);
        Speaker speaker = myRobot.getSpeaker();
        UltrasonicSensor sensor = myRobot.getUltrasonicSensor(Sensor.Port.S1);

        while (true) {
          System.out.println(sensor.getDistance());
        }

        //Disconnect from the Robot
        //myRobot.close();

    }

}
