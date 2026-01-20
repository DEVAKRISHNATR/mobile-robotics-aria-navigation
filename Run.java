import exercises.e1.OdometryModel;
import exercises.e2.ObstacleAvoidance;
import exercises.e3.TargetTracking;


import utils.ControlPanel;
import utils.Delay;

import robot.Robot;

/**
 * Created by Theo Theodoridis.
 * Class    : Run
 * Version  : v1.0
 * Date     : © Copyright 2015
 * User     : ttheod
 * email    : t.theodoridis@salford.ac.uk
 * Comments : To run the main robot class for both, simulator and real robot.
 **/

public class Run
{
    private static Robot robot;
    private static ControlPanel panel;

   /**
    * Method     : Run::RunAria()
    * Purpose    : Default Run class constructor.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public Run()
    {
        robot = new Robot();
        robot.init(robot);
        panel = new ControlPanel(robot, 300);
        panel.ShowGUI();

        // [+]Thread setup:
        update.setPriority(Thread.MAX_PRIORITY);
        update.start();
    }

   /**
    * Thread     : Run::update()
    * Purpose    : To run the update thread.
    * Parameters : None.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    Thread update = new Thread()
    {
       public void run()
       {
           while(true)
           {
               // your code...
               Delay.ms(1);
           }
       }
    };

   /**
    * Method     : Run::main()
    * Purpose    : Default main method to run the robot.Run class.
    * Parameters : - args : Initialization parameters.
    * Returns    : Nothing.
    * Notes      : None.
    **/
    public static void main(String args[])
    {
        new Run();

        /*Exercise 1*/ OdometryModel om = new OdometryModel(robot);
        /*Exercise 2*/ //ObstacleAvoidance oa = new ObstacleAvoidance(robot);
        /*Exercise 3*/ //TargetTracking tt = new TargetTracking(robot);

        while(true)
        {
           /**
            * Move and print test.
            **/
 //           robot.control.turnSmooth(-100);
 //           System.out.printf
 //           (
 //               "\rOdometry: X = %.1f, Y = %.1f, Th = %.1f, " +
 //               "Sensors: s(0) = %.1f, s(3) = %.1f, s(7) = %.1f, " +
 //               "Camera: blob(X) = %d, blob(Y) = %d",
 //               robot.kinematics.getX(), robot.kinematics.getY(), robot.kinematics.getTh(),
 //               robot.sensor.getSonarRange(0), robot.sensor.getSonarRange(3), robot.sensor.getSonarRange(7),
 //               robot.sensor.getBlobX(), robot.sensor.getBlobY()
//            );

           /**
            * Linear/Angular test.
            **/
//            // 1. Linear test:
//            robot.control.move(100);
//            if(om.isLinearDestination(1000, 0))
//            {
//               robot.control.stop();
//                System.exit(0);
//            }
//            // 2. Angular test:
 //           robot.control.turnSpot(-50);
  //          double th = om.get360(robot.kinematics.getTh());
  //          System.out.println("rTh: " + robot.kinematics.getTh() + ", th: " + th);
  //          if(om.isAngularDestination(90))
 //           {
//                robot.control.stop();
//                System.exit(0);
//            }

           /**
            * Exercise 1:
            * Odometry Model test.
            **/
            if(om.navigator(100))
            {
                robot.shutDown();
                System.exit(0);
            }

           /**
            * Exercise 2:
            * Obstacle Avoidance test.
            **/
            //oa.decollide(100);
            //oa.avoid(100);
            //oa.untrap(100);

           /**
            * Exercise 3:
            * Target Tracking test.
            **/
//            tt.approach(100);
//            tt.getKalmanEstimate(robot.sensor.getBlobX());
//            tt.zoneTracker(100);
//            tt.kalmanTracker(100);

            Delay.ms(100);
        }
    }
}
